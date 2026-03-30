#include "http_vr_server.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "frame_jpeg_cache.hpp"
#include "ptz_control.hpp"

namespace {

using json = nlohmann::json;

const char* kManualMode = "manual";
const char* kVrMode = "vr";
constexpr int kSseHeartbeatMs = 2000;

struct ListenerState {
    std::thread thread;
    std::atomic<bool> listen_finished{false};
    std::atomic<bool> listen_result{false};
};

struct EventState;

struct RouteContext {
    HttpVrConfig config;
    std::atomic<bool>* app_running = nullptr;
    FrameJpegCache* frame_cache = nullptr;
    PtzController* ptz = nullptr;
    HttpVrServer::OverlayStateProvider overlay_state_provider;
    EventState* event_state = nullptr;
};

struct EventState {
    std::mutex mutex;
    std::condition_variable cv;
    uint64_t vr_session_toggle_request_id = 0;
};

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

std::string host_without_port(const std::string& host_header) {
    if (host_header.empty()) {
        return "<PI_IP>";
    }

    if (host_header.front() == '[') {
        const size_t end = host_header.find(']');
        if (end != std::string::npos) {
            return host_header.substr(0, end + 1);
        }
    }

    const size_t colon = host_header.find(':');
    return colon == std::string::npos ? host_header : host_header.substr(0, colon);
}

std::string make_https_redirect_target(const httplib::Request& req, int https_port) {
    std::ostringstream oss;
    oss << "https://" << host_without_port(req.get_header_value("Host"));
    if (https_port != 443) {
        oss << ":" << https_port;
    }
    oss << (req.path.empty() ? "/" : req.path);
    return oss.str();
}

template <typename ServerType>
bool start_listener(ServerType* server, ListenerState& listener, int port) {
    listener.listen_finished.store(false);
    listener.listen_result.store(false);
    listener.thread = std::thread([server, &listener, port]() {
        listener.listen_result.store(server->listen("0.0.0.0", port));
        listener.listen_finished.store(true);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (listener.listen_finished.load() && !listener.listen_result.load()) {
        if (listener.thread.joinable()) {
            listener.thread.join();
        }
        return false;
    }
    return true;
}

template <typename ServerType>
void stop_listener(ServerType* server, ListenerState& listener) {
    if (server) {
        server->stop();
    }
    if (listener.thread.joinable()) {
        listener.thread.join();
    }
}

template <typename ServerType>
void register_redirect_routes(ServerType& server, int https_port) {
    auto redirect = [https_port](const httplib::Request& req, httplib::Response& res) {
        res.status = 307;
        res.set_header("Location", make_https_redirect_target(req, https_port));
    };

    server.Get("/", redirect);
    server.Get(R"(/(.*))", redirect);
    server.Post(R"(/(.*))", redirect);
}

} // namespace

struct HttpVrServer::Impl {
    RouteContext ctx;
    EventState event_state;

    std::unique_ptr<httplib::Server> http_server;
    ListenerState http_listener;

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    std::unique_ptr<httplib::SSLServer> https_server;
    ListenerState https_listener;
#endif
};

namespace {

template <typename ServerType>
void register_service_routes(ServerType& server, RouteContext* ctx) {
    server.Get("/health", [](const httplib::Request&, httplib::Response& res) {
        json body = {{"ok", true}};
        res.set_content(body.dump(), "application/json");
    });

    server.Get("/imu/latest", [ctx](const httplib::Request&, httplib::Response& res) {
        const PtzStatus status = ctx->ptz->latest_status();
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

    server.Get("/overlay/state", [ctx](const httplib::Request&, httplib::Response& res) {
        HttpVrOverlayState overlay_state;
        if (ctx->overlay_state_provider) {
            overlay_state = ctx->overlay_state_provider();
        }

        json body = {
            {"rc",
             {
                 {"valid", overlay_state.valid},
                 {"stale", overlay_state.stale},
                 {"x", overlay_state.x},
                 {"y", overlay_state.y},
                 {"yaw_rad", overlay_state.yaw_rad},
                 {"frame", overlay_state.frame},
                 {"ts_ms", overlay_state.ts_ms},
             }},
        };
        res.set_content(body.dump(), "application/json");
    });

    server.Get("/ptz/mode", [ctx](const httplib::Request&, httplib::Response& res) {
        const PtzStatus status = ctx->ptz->latest_status();
        json body = {
            {"mode", status.mode},
            {"servo_ready", status.servo_ready},
        };
        res.set_content(body.dump(), "application/json");
    });

    server.Post("/ptz/mode", [ctx](const httplib::Request& req, httplib::Response& res) {
        try {
            const json payload = json::parse(req.body);
            PtzMode mode = PtzMode::kManual;
            if (!parse_ptz_mode(payload.value("mode", ""), mode)) {
                res.status = 400;
                res.set_content("invalid mode", "text/plain; charset=utf-8");
                return;
            }
            if (!ctx->ptz->set_mode(mode)) {
                res.status = 500;
                res.set_content("ptz not ready", "text/plain; charset=utf-8");
                return;
            }

            const PtzStatus status = ctx->ptz->latest_status();
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

    server.Post("/imu", [ctx](const httplib::Request& req, httplib::Response& res) {
        try {
            const json payload = json::parse(req.body);
            if (payload.value("type", "") == "imu") {
                ctx->ptz->handle_imu(payload.value("pitch", 0.0f),
                                     payload.value("roll", 0.0f),
                                     payload.value("yaw", 0.0f),
                                     payload.value("t", static_cast<uint64_t>(0)));
            }
            res.status = 204;
        } catch (const std::exception& exc) {
            res.status = 400;
            res.set_content(std::string("bad json: ") + exc.what(),
                            "text/plain; charset=utf-8");
        }
    });

    server.Get("/events", [ctx](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_header("Pragma", "no-cache");
        res.set_header("Connection", "keep-alive");
        res.set_header("X-Accel-Buffering", "no");
        res.set_chunked_content_provider(
            "text/event-stream; charset=utf-8",
            [ctx](size_t, httplib::DataSink& sink) {
                uint64_t last_sent_request = 0;
                {
                    std::lock_guard<std::mutex> lock(ctx->event_state->mutex);
                    last_sent_request = ctx->event_state->vr_session_toggle_request_id;
                }

                auto next_heartbeat =
                    std::chrono::steady_clock::now() + std::chrono::milliseconds(kSseHeartbeatMs);

                while (ctx->app_running->load()) {
                    uint64_t pending_request = last_sent_request;
                    {
                        std::unique_lock<std::mutex> lock(ctx->event_state->mutex);
                        ctx->event_state->cv.wait_until(lock, next_heartbeat, [&]() {
                            return !ctx->app_running->load() ||
                                   ctx->event_state->vr_session_toggle_request_id > last_sent_request;
                        });
                        pending_request = ctx->event_state->vr_session_toggle_request_id;
                    }

                    if (!ctx->app_running->load()) {
                        return false;
                    }

                    if (pending_request > last_sent_request) {
                        std::ostringstream event;
                        event << "event: vr_session_toggle\n"
                              << "data: {\"request_id\":" << pending_request << "}\n\n";
                        const std::string payload = event.str();
                        if (!sink.write(payload.data(), payload.size())) {
                            return false;
                        }
                        last_sent_request = pending_request;
                        next_heartbeat =
                            std::chrono::steady_clock::now() + std::chrono::milliseconds(kSseHeartbeatMs);
                        continue;
                    }

                    static const std::string heartbeat = ": keepalive\n\n";
                    if (!sink.write(heartbeat.data(), heartbeat.size())) {
                        return false;
                    }
                    next_heartbeat =
                        std::chrono::steady_clock::now() + std::chrono::milliseconds(kSseHeartbeatMs);
                }
                return false;
            });
    });

    server.Get("/stream.mjpg", [ctx](const httplib::Request&, httplib::Response& res) {
        ctx->frame_cache->add_consumer();
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_header("Pragma", "no-cache");
        res.set_header("Connection", "close");
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=frame",
            [ctx](size_t, httplib::DataSink& sink) {
                static thread_local uint64_t last_frame = 0;
                while (ctx->app_running->load()) {
                    std::vector<unsigned char> jpg;
                    uint64_t frame_id = 0;
                    if (!ctx->frame_cache->latest_jpeg(jpg, &frame_id) || frame_id == last_frame) {
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
            [ctx](bool) {
                ctx->frame_cache->remove_consumer();
            });
    });

    server.Get("/", [](const httplib::Request&, httplib::Response& res) {
        res.status = 302;
        res.set_header("Location", "/web/");
    });

    server.Get("/web", [](const httplib::Request&, httplib::Response& res) {
        res.status = 302;
        res.set_header("Location", "/web/");
    });

    server.Get(R"(/web/$)", [ctx](const httplib::Request&, httplib::Response& res) {
        std::string body;
        if (!load_file_text(std::filesystem::path(ctx->config.web_root) / "index.html", body)) {
            res.status = 404;
            return;
        }
        res.set_content(body, "text/html; charset=utf-8");
    });

    server.Get(R"(/web/(.+))", [ctx](const httplib::Request& req, httplib::Response& res) {
        const std::string rel = req.matches[1].str();
        if (rel.find("..") != std::string::npos) {
            res.status = 400;
            return;
        }

        const auto file_path = std::filesystem::path(ctx->config.web_root) / rel;
        std::string body;
        if (!load_file_text(file_path, body)) {
            res.status = 404;
            return;
        }
        res.set_content(body, mime_type_for(file_path.extension().string()).c_str());
    });
}

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
bool tls_files_ready(const HttpVrConfig& config) {
    return std::filesystem::exists(config.tls_cert_file) &&
           std::filesystem::exists(config.tls_key_file);
}
#endif

} // namespace

HttpVrServer::~HttpVrServer() {
    stop();
}

bool HttpVrServer::start(const HttpVrConfig& cfg,
                         std::atomic<bool>& app_running,
                         FrameJpegCache& frame_cache,
                         PtzController& ptz_controller,
                         OverlayStateProvider overlay_state_provider) {
    if (impl_) {
        return true;
    }

    auto* impl = new Impl();
    impl->ctx.config = cfg;
    impl->ctx.app_running = &app_running;
    impl->ctx.frame_cache = &frame_cache;
    impl->ctx.ptz = &ptz_controller;
    impl->ctx.overlay_state_provider = std::move(overlay_state_provider);
    impl->ctx.event_state = &impl->event_state;

    const bool https_requested = cfg.https_enable;
    bool https_active = false;

    if (https_requested && cfg.redirect_http_to_https && cfg.port == cfg.https_port) {
        std::fprintf(stderr,
                     "[HTTP] redirect_http_to_https requires different HTTP/HTTPS ports (%d).\n",
                     cfg.port);
        delete impl;
        return false;
    }

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    if (https_requested) {
        if (!tls_files_ready(cfg)) {
            std::fprintf(stderr,
                         "[HTTP] HTTPS requested but TLS cert/key missing (%s, %s). Falling back to HTTP.\n",
                         cfg.tls_cert_file.c_str(),
                         cfg.tls_key_file.c_str());
        } else {
            impl->https_server =
                std::make_unique<httplib::SSLServer>(cfg.tls_cert_file.c_str(), cfg.tls_key_file.c_str());
            if (!impl->https_server->is_valid()) {
                std::fprintf(stderr,
                             "[HTTP] HTTPS requested but SSLServer init failed. Falling back to HTTP.\n");
                impl->https_server.reset();
            } else {
                register_service_routes(*impl->https_server, &impl->ctx);
                if (!start_listener(impl->https_server.get(), impl->https_listener, cfg.https_port)) {
                    std::fprintf(stderr,
                                 "[HTTP] HTTPS listen failed on port %d.\n",
                                 cfg.https_port);
                    delete impl;
                    return false;
                }
                https_active = true;
            }
        }
    }
#else
    if (https_requested) {
        std::fprintf(stderr,
                     "[HTTP] HTTPS requested but this build has no OpenSSL support. Falling back to HTTP.\n");
    }
#endif

    impl->http_server = std::make_unique<httplib::Server>();
    if (https_active && cfg.redirect_http_to_https) {
        register_redirect_routes(*impl->http_server, cfg.https_port);
    } else {
        register_service_routes(*impl->http_server, &impl->ctx);
    }

    if (!start_listener(impl->http_server.get(), impl->http_listener, cfg.port)) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
        stop_listener(impl->https_server.get(), impl->https_listener);
#endif
        stop_listener(impl->http_server.get(), impl->http_listener);
        delete impl;
        return false;
    }

    impl_ = impl;
    if (https_active && cfg.redirect_http_to_https) {
        std::fprintf(stderr,
                     "[HTTP] HTTP redirect ready: http://<PI_IP>:%d -> https://<PI_IP>:%d\n",
                     cfg.port,
                     cfg.https_port);
        std::fprintf(stderr,
                     "[HTTP] HTTPS tiltVR ready: https://<PI_IP>:%d/web/\n",
                     cfg.https_port);
    } else if (https_active) {
        std::fprintf(stderr, "[HTTP] tiltVR web ready: http://<PI_IP>:%d/web/\n", cfg.port);
        std::fprintf(stderr,
                     "[HTTP] HTTPS tiltVR ready: https://<PI_IP>:%d/web/\n",
                     cfg.https_port);
    } else {
        std::fprintf(stderr, "[HTTP] tiltVR web ready: http://<PI_IP>:%d/web/\n", cfg.port);
    }
    return true;
}

void HttpVrServer::publish_vr_session_toggle_request() {
    if (!impl_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->event_state.mutex);
        ++impl_->event_state.vr_session_toggle_request_id;
    }
    impl_->event_state.cv.notify_all();
}

void HttpVrServer::stop() {
    if (!impl_) {
        return;
    }

    impl_->event_state.cv.notify_all();

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    stop_listener(impl_->https_server.get(), impl_->https_listener);
#endif
    stop_listener(impl_->http_server.get(), impl_->http_listener);

    delete impl_;
    impl_ = nullptr;
}
