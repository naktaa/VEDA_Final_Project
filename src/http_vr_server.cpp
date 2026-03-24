#include "http_vr_server.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "frame_jpeg_cache.hpp"
#include "ptz_control.hpp"
#include "vr_ui_command_bridge.hpp"

namespace {

using json = nlohmann::json;

const char* kManualMode = "manual";
const char* kVrMode = "vr";

bool parse_ptz_mode(const std::string& mode_text, PtzMode& mode) {
    if (mode_text == kManualMode) {
        mode = PtzMode::kManual;
        return true;
    }
    if (mode_text == kVrMode) {
        mode = PtzMode::kVr;
        return true;
    }
    return false;
}

std::string mime_type_for(const std::string& ext) {
    static const std::unordered_map<std::string, std::string> kMimeTypes = {
        {".html", "text/html; charset=utf-8"},
        {".js", "application/javascript; charset=utf-8"},
        {".css", "text/css; charset=utf-8"},
        {".json", "application/json; charset=utf-8"},
        {".png", "image/png"},
        {".jpg", "image/jpeg"},
        {".jpeg", "image/jpeg"},
        {".svg", "image/svg+xml"},
        {".ico", "image/x-icon"},
    };
    const auto it = kMimeTypes.find(ext);
    return it == kMimeTypes.end() ? "application/octet-stream" : it->second;
}

bool load_file_text(const std::filesystem::path& path, std::string& out) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        return false;
    }

    std::ostringstream buffer;
    buffer << ifs.rdbuf();
    out = buffer.str();
    return true;
}

} // namespace

struct HttpVrServer::Impl {
    HttpVrConfig config;
    std::atomic<bool>* app_running = nullptr;
    FrameJpegCache* frame_cache = nullptr;
    PtzController* ptz = nullptr;
    VrUiCommandBridge* vr_ui_command_bridge = nullptr;
    std::unique_ptr<httplib::Server> server;
    std::thread server_thread;
    std::atomic<bool> listen_finished{false};
    std::atomic<bool> listen_result{false};
};

HttpVrServer::~HttpVrServer() {
    stop();
}

bool HttpVrServer::start(const HttpVrConfig& cfg,
                         std::atomic<bool>& app_running,
                         FrameJpegCache& frame_cache,
                         PtzController& ptz_controller,
                         VrUiCommandBridge& vr_ui_command_bridge) {
    if (impl_) {
        return true;
    }

    auto* impl = new Impl();
    impl->config = cfg;
    impl->app_running = &app_running;
    impl->frame_cache = &frame_cache;
    impl->ptz = &ptz_controller;
    impl->vr_ui_command_bridge = &vr_ui_command_bridge;
    impl->server = std::make_unique<httplib::Server>();

    impl->server->Get("/health", [](const httplib::Request&, httplib::Response& res) {
        json body = {{"ok", true}};
        res.set_content(body.dump(), "application/json");
    });

    impl->server->Get("/imu/latest", [impl](const httplib::Request&, httplib::Response& res) {
        const PtzStatus status = impl->ptz->latest_status();
        json body = {
            {"mode", status.mode},
            {"pitch", status.pitch},
            {"roll", status.roll},
            {"yaw", status.yaw},
            {"t", status.client_timestamp_ms},
            {"servo_ready", status.servo_ready},
            {"pan", status.pan},
            {"tilt", status.tilt},
            {"source", status.active_source},
        };
        res.set_content(body.dump(), "application/json");
    });

    impl->server->Get("/ptz/mode", [impl](const httplib::Request&, httplib::Response& res) {
        const PtzStatus status = impl->ptz->latest_status();
        json body = {
            {"mode", status.mode},
            {"servo_ready", status.servo_ready},
        };
        res.set_content(body.dump(), "application/json");
    });

    impl->server->Post("/ptz/mode", [impl](const httplib::Request& req, httplib::Response& res) {
        try {
            const json payload = json::parse(req.body);
            PtzMode mode = PtzMode::kManual;
            if (!parse_ptz_mode(payload.value("mode", ""), mode)) {
                res.status = 400;
                res.set_content("invalid mode", "text/plain; charset=utf-8");
                return;
            }
            if (!impl->ptz->set_mode(mode)) {
                res.status = 500;
                res.set_content("ptz not ready", "text/plain; charset=utf-8");
                return;
            }

            const PtzStatus status = impl->ptz->latest_status();
            json body = {
                {"ok", true},
                {"mode", status.mode},
                {"servo_ready", status.servo_ready},
            };
            res.set_content(body.dump(), "application/json");
        } catch (const std::exception& exc) {
            res.status = 400;
            res.set_content(std::string("bad json: ") + exc.what(),
                            "text/plain; charset=utf-8");
        }
    });

    impl->server->Post("/imu", [impl](const httplib::Request& req, httplib::Response& res) {
        try {
            const json payload = json::parse(req.body);
            if (payload.value("type", "") == "imu") {
                const float pitch = payload.value("pitch", 0.0f);
                const float roll = payload.value("roll", 0.0f);
                const float yaw = payload.value("yaw", 0.0f);
                const uint64_t t = payload.value("t", static_cast<uint64_t>(0));
                static thread_local int imu_log_count = 0;
                if (imu_log_count < 5) {
                    std::fprintf(stderr,
                                 "[HTTP] /imu pitch=%.1f roll=%.1f yaw=%.1f t=%llu\n",
                                 pitch,
                                 roll,
                                 yaw,
                                 static_cast<unsigned long long>(t));
                    ++imu_log_count;
                }
                impl->ptz->handle_imu(pitch, roll, yaw, t);
            }
            res.status = 204;
        } catch (const std::exception& exc) {
            res.status = 400;
            res.set_content(std::string("bad json: ") + exc.what(),
                            "text/plain; charset=utf-8");
        }
    });

    impl->server->Get("/vr/ui-commands", [impl](const httplib::Request&, httplib::Response& res) {
        const VrUiCommandSnapshot snapshot = impl->vr_ui_command_bridge->snapshot();
        json body = {
            {"ok", true},
            {"session_start_seq", snapshot.session_start_seq},
            {"session_stop_seq", snapshot.session_stop_seq},
            {"zero_calibrate_seq", snapshot.zero_calibrate_seq},
            {"session_active", snapshot.session_active},
            {"vr_mode_active", snapshot.vr_mode_active},
        };
        res.set_content(body.dump(), "application/json");
    });

    impl->server->Post("/vr/session-state", [impl](const httplib::Request& req, httplib::Response& res) {
        try {
            const json payload = json::parse(req.body);
            const bool active = payload.value("active", false);
            const bool vr_mode_active = payload.value("vr_mode_active", false);
            impl->vr_ui_command_bridge->set_state(active, vr_mode_active);
            json body = {
                {"ok", true},
                {"active", active},
                {"vr_mode_active", vr_mode_active},
            };
            res.set_content(body.dump(), "application/json");
        } catch (const std::exception& exc) {
            res.status = 400;
            res.set_content(std::string("bad json: ") + exc.what(),
                            "text/plain; charset=utf-8");
        }
    });

    impl->server->Get("/stream.mjpg", [impl](const httplib::Request&, httplib::Response& res) {
        impl->frame_cache->add_consumer();
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_header("Pragma", "no-cache");
        res.set_header("Connection", "close");
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=frame",
            [impl](size_t, httplib::DataSink& sink) {
                static thread_local uint64_t last_frame = 0;
                while (impl->app_running->load()) {
                    std::vector<unsigned char> jpg;
                    uint64_t frame_id = 0;
                    if (!impl->frame_cache->latest_jpeg(jpg, &frame_id) || frame_id == last_frame) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                    last_frame = frame_id;

                    std::ostringstream header;
                    header << "--frame\r\n"
                           << "Content-Type: image/jpeg\r\n"
                           << "Content-Length: " << jpg.size() << "\r\n\r\n";
                    const std::string header_text = header.str();

                    if (!sink.write(header_text.data(), header_text.size())) {
                        return false;
                    }
                    if (!sink.write(reinterpret_cast<const char*>(jpg.data()), jpg.size())) {
                        return false;
                    }
                    if (!sink.write("\r\n", 2)) {
                        return false;
                    }
                }
                return false;
            },
            [impl](bool) {
                impl->frame_cache->remove_consumer();
            });
    });

    impl->server->Get("/", [](const httplib::Request&, httplib::Response& res) {
        res.status = 302;
        res.set_header("Location", "/web/");
    });

    impl->server->Get("/web", [](const httplib::Request&, httplib::Response& res) {
        res.status = 302;
        res.set_header("Location", "/web/");
    });

    impl->server->Get(R"(/web/$)", [impl](const httplib::Request&, httplib::Response& res) {
        std::string body;
        if (!load_file_text(std::filesystem::path(impl->config.web_root) / "index.html", body)) {
            res.status = 404;
            return;
        }
        res.set_content(body, "text/html; charset=utf-8");
    });

    impl->server->Get(R"(/web/(.+))", [impl](const httplib::Request& req, httplib::Response& res) {
        const std::string rel = req.matches[1].str();
        if (rel.find("..") != std::string::npos) {
            res.status = 400;
            return;
        }

        const auto file_path = std::filesystem::path(impl->config.web_root) / rel;
        std::string body;
        if (!load_file_text(file_path, body)) {
            res.status = 404;
            return;
        }
        res.set_content(body, mime_type_for(file_path.extension().string()).c_str());
    });

    impl->server_thread = std::thread([impl]() {
        impl->listen_result.store(impl->server->listen("0.0.0.0", impl->config.port));
        impl->listen_finished.store(true);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (impl->listen_finished.load() && !impl->listen_result.load()) {
        if (impl->server_thread.joinable()) {
            impl->server_thread.join();
        }
        delete impl;
        return false;
    }

    impl_ = impl;
    std::fprintf(stderr, "[HTTP] tiltVR web ready: http://<PI_IP>:%d/web/\n", cfg.port);
    return true;
}

void HttpVrServer::stop() {
    if (!impl_) {
        return;
    }

    if (impl_->server) {
        impl_->server->stop();
    }
    if (impl_->server_thread.joinable()) {
        impl_->server_thread.join();
    }

    delete impl_;
    impl_ = nullptr;
}
