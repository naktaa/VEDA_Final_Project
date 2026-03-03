#include <mosquitto.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <filesystem>
#include <regex>
#include <atomic>
#include <thread>
#include <chrono>
#include <sstream>
#include <optional>
#include <ctime>

namespace fs = std::filesystem;
static std::string today_ymd_local()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);  // ? 시스템 로컬 타임존 기준

    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
    return std::string(buf);
}
// ------------------------- Config -------------------------
static const std::string ROOT_DIR  = "/home/pi/final_veda_test";
static const std::string CLIP_DIR  = ROOT_DIR + "/clips";
static const int         CLIP_SEC  = 5;          // ? 발생 시점부터 5초
static const int         CLIP_TTL_SEC = 86400;   // ? 24h (원하면 조정)
static const int         GC_INTERVAL_MS = 5000;

static const char* MQTT_HOST  = "127.0.0.1";
static const int   MQTT_PORT  = 1883;
static const char* MQTT_TOPIC = "wiserisk/cctv/event";

static const int   HTTP_PORT  = 1885;            // ? python http server port
static const std::string SERVER_IP = "192.168.100.10"; // ? 라즈베리파이 IP로 고정

static const std::string CAM_ID = "cam01";
static const std::string RTSP_URL =
    "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";

// ------------------------- Utils -------------------------
static bool ensure_dir(const std::string& path) {
    std::error_code ec;
    if (fs::exists(path, ec)) return fs::is_directory(path, ec);
    return fs::create_directories(path, ec);
}

static std::optional<long long> file_mtime_epoch_sec(const std::string& path) {
    std::error_code ec;
    if (!fs::exists(path, ec)) return std::nullopt;
    auto ftime = fs::last_write_time(path, ec);
    if (ec) return std::nullopt;

    auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now()
    );
    return std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count();
}

static bool remove_file(const std::string& path) {
    std::error_code ec;
    return fs::remove(path, ec);
}

static bool is_safe_id(const std::string& s) {
    static const std::regex re(R"(^[A-Za-z0-9._-]{1,128}$)");
    return std::regex_match(s, re);
}

static std::string json_escape(const std::string& s) {
    std::string o;
    o.reserve(s.size() + 16);
    for (char c : s) {
        switch (c) {
            case '\\': o += "\\\\"; break;
            case '"':  o += "\\\""; break;
            case '\n': o += "\\n";  break;
            case '\r': o += "\\r";  break;
            case '\t': o += "\\t";  break;
            default:   o += c;      break;
        }
    }
    return o;
}

static inline std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    size_t e = s.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    return s.substr(b, e - b + 1);
}

static bool find_between(const std::string& src,
                         const std::string& left,
                         const std::string& right,
                         std::string& out,
                         size_t startPos = 0)
{
    size_t p1 = src.find(left, startPos);
    if (p1 == std::string::npos) return false;
    p1 += left.size();
    size_t p2 = src.find(right, p1);
    if (p2 == std::string::npos) return false;
    out = src.substr(p1, p2 - p1);
    return true;
}

static bool find_attr_value(const std::string& src,
                            const std::string& key,
                            std::string& out,
                            size_t startPos = 0)
{
    size_t p = src.find(key, startPos);
    if (p == std::string::npos) return false;
    p += key.size();
    size_t q = src.find("\"", p);
    if (q == std::string::npos) return false;
    out = src.substr(p, q - p);
    return true;
}

static std::string extract_simpleitem_value(const std::string& xml,
                                            const std::string& name)
{
    std::string key = "Name=\"" + name + "\"";
    size_t p = xml.find(key);
    if (p == std::string::npos) return "";
    std::string out;
    if (!find_attr_value(xml, "Value=\"", out, p)) return "";
    return out;
}

static std::string extract_topic_full(const std::string& xml) {
    std::string out;
    if (!find_between(xml, "<wsnt:Topic", "</wsnt:Topic>", out)) return "";
    size_t gt = out.find(">");
    if (gt == std::string::npos) return "";
    return trim(out.substr(gt + 1));
}

static std::string last_token(const std::string& topic_full) {
    auto pos = topic_full.find_last_of('/');
    std::string t = (pos == std::string::npos) ? topic_full : topic_full.substr(pos + 1);
    auto colon = t.find_last_of(':');
    if (colon != std::string::npos) t = t.substr(colon + 1);
    return t;
}

static bool is_interesting_event(const std::string& eventName) {
    return (eventName == "IvaArea" ||
            eventName == "MaskDetection" ||
            eventName == "SlipAndFallDetection" ||
            eventName == "ObjectCounterAlarm");
}

// ------------------------- ClipGC -------------------------
class ClipGC {
public:
    ClipGC(std::string clip_root_dir, int ttl_sec, int interval_ms)
        : clip_root_dir_(std::move(clip_root_dir)),
          ttl_sec_(ttl_sec),
          interval_ms_(interval_ms) {}

    ~ClipGC() { stop(); }

    void start() {
        if (running_) return;
        running_ = true;
        th_ = std::thread([this]{ loop(); });
    }

    void stop() {
        running_ = false;
        if (th_.joinable()) th_.join();
    }

private:
    void loop() {
        while (running_) {
            std::error_code ec;
            if (fs::exists(clip_root_dir_, ec)) {
                auto now = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

                for (auto& p : fs::recursive_directory_iterator(clip_root_dir_, ec)) {
                    if (ec) break;
                    if (!p.is_regular_file(ec)) continue;
                    if (p.path().extension() != ".mp4") continue;

                    auto mt = file_mtime_epoch_sec(p.path().string());
                    if (!mt.has_value()) continue;
                    if (now - *mt >= ttl_sec_) {
                        remove_file(p.path().string());
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
        }
    }

    std::string clip_root_dir_;
    int ttl_sec_;
    int interval_ms_;
    std::atomic<bool> running_{false};
    std::thread th_;
};

// ------------------------- Capture clip -------------------------
static bool capture_clip_mp4_5s(const std::string& out_path_mp4) {
    // ? 임시파일은 반드시 ".tmp.mp4"로 (확장자가 mp4로 끝나야 ffmpeg가 포맷을 앎)
    const std::string tmp_path = out_path_mp4 + ".tmp.mp4";

    // 이전 tmp 찌꺼기 제거
    {
        std::error_code ec;
        if (fs::exists(tmp_path, ec)) fs::remove(tmp_path, ec);
    }

    auto exists_and_big_enough = [&](const std::string& p) -> bool {
        std::error_code ec;
        if (!fs::exists(p, ec) || !fs::is_regular_file(p, ec)) return false;
        auto sz = fs::file_size(p, ec);
        if (ec) return false;
        return (sz > 1024); // 1KB 이상이면 일단 "생성됨"으로 간주(원하면 더 키워도 됨)
    };

    auto finalize = [&]() -> bool {
        // tmp가 정상 생성되면 최종 파일로 rename
        if (!exists_and_big_enough(tmp_path)) return false;

        std::error_code ec;
        // 기존 파일 있으면 지우고 교체(atomic 느낌으로)
        if (fs::exists(out_path_mp4, ec)) fs::remove(out_path_mp4, ec);

        fs::rename(tmp_path, out_path_mp4, ec);
        if (ec) {
            // rename 실패 시 tmp 유지될 수 있으니 정리
            std::error_code ec2;
            fs::remove(tmp_path, ec2);
            return false;
        }
        return true;
    };

    // 1) copy 우선
    {
        std::ostringstream cmd;
        cmd << "ffmpeg -y -hide_banner -loglevel error "
            << "-rtsp_transport tcp "
            << "-i \"" << RTSP_URL << "\" "
            << "-t " << CLIP_SEC << " "
            << "-an -c copy "
            << "-movflags +faststart "
            << "-reset_timestamps 1 "
            << "\"" << tmp_path << "\"";

        int ret = std::system(cmd.str().c_str());
        if (ret == 0 && finalize()) return true;

        // 실패 시 tmp 정리
        std::error_code ec;
        if (fs::exists(tmp_path, ec)) fs::remove(tmp_path, ec);
    }

    // 2) fallback re-encode
    {
        std::ostringstream cmd;
        cmd << "ffmpeg -y -hide_banner -loglevel error "
            << "-rtsp_transport tcp "
            << "-i \"" << RTSP_URL << "\" "
            << "-t " << CLIP_SEC << " "
            << "-an -c:v libx264 -preset veryfast "
            << "-movflags +faststart "
            << "-reset_timestamps 1 "
            << "\"" << tmp_path << "\"";

        int ret = std::system(cmd.str().c_str());
        if (ret == 0 && finalize()) return true;

        // 실패 시 tmp 정리
        std::error_code ec;
        if (fs::exists(tmp_path, ec)) fs::remove(tmp_path, ec);
    }

    return false;
}


// ------------------------- Parse XML -> fields -------------------------
struct ParsedEv {
    std::string topic_full;
    std::string topic;
    std::string utc;
    std::string rule;
    std::string action;
    std::string objectId;
    std::string state_str;
};

static bool parse_one_xml(const std::string& xml, ParsedEv& out) {
    out.topic_full = extract_topic_full(xml);
    if (out.topic_full.empty()) return false;

    find_attr_value(xml, "UtcTime=\"", out.utc);

    out.rule      = extract_simpleitem_value(xml, "RuleName");
    out.state_str = extract_simpleitem_value(xml, "State");
    out.objectId  = extract_simpleitem_value(xml, "ObjectId");
    out.action    = extract_simpleitem_value(xml, "Action");

    out.topic = last_token(out.topic_full);
    return true;
}

static std::string make_event_id(const ParsedEv& ev) {
    std::string utc = ev.utc;
    for (char& c : utc) {
        if (c == ':' || c == '/' || c == ' ') c = '_';
    }
    std::string oid = ev.objectId.empty() ? "noobj" : ev.objectId;

    std::string id = CAM_ID + "_" + ev.topic + "_" + utc + "_" + oid;

    std::string safe;
    safe.reserve(id.size());
    for (char c : id) {
        if ((c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '_' || c == '-' || c == '.')
            safe.push_back(c);
    }
    if (safe.size() > 120) safe.resize(120);
    if (!is_safe_id(safe)) return "event_invalid";
    return safe;
}

// ------------------------- python http server runner -------------------------
static FILE* start_python_http_server() {
    // ROOT_DIR 를 document root로 두면 /clips/xxx.mp4 접근 가능
    std::ostringstream cmd;
    cmd << "python3 -m http.server " << HTTP_PORT
        << " --directory \"" << ROOT_DIR << "\""
        << " >/dev/null 2>&1";

    // 백그라운드로 띄우기 위해 sh -c "... &"
    std::string shcmd = "sh -c '" + cmd.str() + " &'";
    return popen(shcmd.c_str(), "r");
}

// ------------------------- MQTT publish -------------------------
static void publish_mqtt(mosquitto* mosq, const std::string& payload) {
    mosquitto_publish(mosq, nullptr, MQTT_TOPIC,
                      (int)payload.size(), payload.c_str(),
                      0, false);
    mosquitto_loop(mosq, 0, 1);
}

// ------------------------- main -------------------------
int main(int, char**)
{
    ensure_dir(ROOT_DIR);
    ensure_dir(CLIP_DIR);

    // ? ClipGC 실행
    ClipGC clip_gc(CLIP_DIR, CLIP_TTL_SEC, GC_INTERVAL_MS);
    clip_gc.start();

    // ? python http server 실행 (1885)
    // (이미 떠 있으면 중복 실행될 수도 있음 → 필요하면 pkill로 정리하도록 해줄게)
    //start_python_http_server();

    // ? MQTT init
    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new(nullptr, true, nullptr);
    if (!mosq) {
        std::cerr << "mosquitto_new failed\n";
        return 1;
    }
    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "mosquitto_connect failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // ? ffmpeg metadata stream parse
    std::string cmd =
        "ffmpeg -loglevel error -rtsp_transport tcp "
        "-i \"" + RTSP_URL + "\" "
        "-map 0:1 -c copy -f data -";

    FILE* fp = popen(cmd.c_str(), "r");
    if (!fp) {
        std::cerr << "popen(ffmpeg) failed\n";
        clip_gc.stop();
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    std::string buf;
    buf.reserve(1024 * 1024);

    char readbuf[8192];
    while (true) {
        size_t n = fread(readbuf, 1, sizeof(readbuf), fp);
        if (n == 0) break;
        buf.append(readbuf, n);

        while (true) {
            size_t p1 = buf.find("<?xml");
            if (p1 == std::string::npos) break;
            size_t p2 = buf.find("<?xml", p1 + 5);
            if (p2 == std::string::npos) break;

            std::string one = buf.substr(p1, p2 - p1);
            buf.erase(0, p2);

            ParsedEv ev{};
            if (!parse_one_xml(one, ev)) continue;

            // ? interesting 이벤트만
            if (!is_interesting_event(ev.topic)) continue;

            // ? state bool
            bool state = (ev.state_str == "true");

            // ? clip은 state=true에서만
            std::string clip_url = "";
            if (state) {
                std::string event_id = make_event_id(ev);
                std::string date_dir = CLIP_DIR + "/" + today_ymd_local();
                ensure_dir(date_dir);
                
                std::string out_mp4 = date_dir + "/" + event_id + ".mp4";

                if (capture_clip_mp4_5s(out_mp4)) {
                    // ? 파일이 실제로 완성될 시간을 아주 조금 준다(레이스 방지)
                    std::this_thread::sleep_for(std::chrono::milliseconds(120));
                
                    std::error_code ec;
                    auto sz = fs::file_size(out_mp4, ec);
                    if (!ec && sz > 1024) {
                        std::string ymd = today_ymd_local();
                        clip_url = "http://" + SERVER_IP + ":" + std::to_string(HTTP_PORT)
                                 + "/clips/" + ymd + "/" + event_id + ".mp4";
                    } else {
                        std::cerr << "[CLIP] file too small or size check failed\n";
                    }
                }

            }

            std::ostringstream j;
            j << "{"
              << "\"src\":\"cctv\","                 // ?? 여기 추가
              << "\"cam\":\"" << json_escape(CAM_ID) << "\","
              << "\"topic\":\"" << json_escape(ev.topic) << "\","
              << "\"topic_full\":\"" << json_escape(ev.topic_full) << "\","
              << "\"utc\":\"" << json_escape(ev.utc) << "\","
              << "\"rule\":\"" << json_escape(ev.rule) << "\","
              << "\"action\":\"" << json_escape(ev.action) << "\","
              << "\"objectId\":\"" << json_escape(ev.objectId) << "\","
              << "\"state\":" << (state ? "true" : "false") << ","
              << "\"clip_sec\":" << CLIP_SEC;


            if (!clip_url.empty()) {
                j << ",\"clip_url\":\"" << json_escape(clip_url) << "\"";
            }
            j << "}";

            publish_mqtt(mosq, j.str());
            std::cout << j.str() << std::endl;
        }

        if (buf.size() > 8 * 1024 * 1024) {
            buf.erase(0, buf.size() - 1024 * 1024);
        }
    }

    pclose(fp);

    clip_gc.stop();

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
