#include <mosquitto.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace {
constexpr const char* kMqttTopic = "wiserisk/cctv/event";
constexpr const char* kDefaultMqttHost = "127.0.0.1";
constexpr int kDefaultMqttPort = 1883;
constexpr int kDefaultHttpPort = 1885;
constexpr int kDefaultClipSec = 5;
constexpr int kDefaultClipTtlSec = 86400;
constexpr int kDefaultGcIntervalMs = 5000;
constexpr int kDefaultTargetFrameW = 1920;
constexpr int kDefaultTargetFrameH = 1080;
constexpr double kSyntheticIdQuantPx = 48.0;

struct RuntimeConfig {
    std::string root_dir = "/home/pi/final_veda_test";
    std::string clip_dir = root_dir + "/clips";
    std::string mqtt_host = kDefaultMqttHost;
    int mqtt_port = kDefaultMqttPort;
    int http_port = kDefaultHttpPort;
    int clip_sec = kDefaultClipSec;
    int clip_ttl_sec = kDefaultClipTtlSec;
    int gc_interval_ms = kDefaultGcIntervalMs;
    std::string server_ip = "192.168.100.10";
    std::string cam_id = "cam01";
    std::string rtsp_url = "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
    int target_frame_w = kDefaultTargetFrameW;
    int target_frame_h = kDefaultTargetFrameH;
};

struct SimpleItem {
    std::string name;
    std::string value;
};

struct BoundingBox {
    double left = 0.0;
    double top = 0.0;
    double right = 0.0;
    double bottom = 0.0;

    bool valid() const {
        return std::isfinite(left) && std::isfinite(top) &&
               std::isfinite(right) && std::isfinite(bottom) &&
               right > left && bottom > top;
    }
};

struct ParsedEv {
    std::string topic_full;
    std::string topic;
    std::string utc;
    std::string rule;
    std::string action;
    std::string object_id;
    std::string object_type;
    std::string class_types;
    std::string state_str;
    std::string property_operation;
    BoundingBox bbox;
    bool has_bbox = false;
    int frame_w = 0;
    int frame_h = 0;
};

std::tm localtime_safe(std::time_t t)
{
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    return tm;
}

std::string today_ymd_local()
{
    const std::time_t t = std::time(nullptr);
    const std::tm tm = localtime_safe(t);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
    return std::string(buf);
}

bool ensure_dir(const std::string& path)
{
    std::error_code ec;
    if (fs::exists(path, ec)) return fs::is_directory(path, ec);
    return fs::create_directories(path, ec);
}

std::optional<long long> file_mtime_epoch_sec(const std::string& path)
{
    std::error_code ec;
    if (!fs::exists(path, ec)) return std::nullopt;
    const auto ftime = fs::last_write_time(path, ec);
    if (ec) return std::nullopt;

    const auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
    return std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count();
}

bool remove_file(const std::string& path)
{
    std::error_code ec;
    return fs::remove(path, ec);
}

bool is_safe_id(const std::string& s)
{
    static const std::regex re(R"(^[A-Za-z0-9._-]{1,128}$)");
    return std::regex_match(s, re);
}

std::string json_escape(const std::string& s)
{
    std::string out;
    out.reserve(s.size() + 16);
    for (char c : s) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out += c; break;
        }
    }
    return out;
}

std::string trim(const std::string& s)
{
    const size_t begin = s.find_first_not_of(" \t\r\n");
    const size_t end = s.find_last_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    return s.substr(begin, end - begin + 1);
}

bool find_between(const std::string& src,
                  const std::string& left,
                  const std::string& right,
                  std::string& out,
                  size_t start_pos = 0)
{
    size_t p1 = src.find(left, start_pos);
    if (p1 == std::string::npos) return false;
    p1 += left.size();
    const size_t p2 = src.find(right, p1);
    if (p2 == std::string::npos) return false;
    out = src.substr(p1, p2 - p1);
    return true;
}

bool find_attr_value(const std::string& src,
                     const std::string& key,
                     std::string& out,
                     size_t start_pos = 0)
{
    size_t p = src.find(key, start_pos);
    if (p == std::string::npos) return false;
    p += key.size();
    const size_t q = src.find('"', p);
    if (q == std::string::npos) return false;
    out = src.substr(p, q - p);
    return true;
}

std::vector<SimpleItem> extract_simple_items(const std::string& xml)
{
    static const std::regex re("Name=\"([^\"]+)\"\\s+Value=\"([^\"]*)\"");
    std::vector<SimpleItem> out;
    for (std::sregex_iterator it(xml.begin(), xml.end(), re), end; it != end; ++it) {
        out.push_back({(*it)[1].str(), (*it)[2].str()});
    }
    return out;
}

std::string normalize_key(const std::string& value)
{
    std::string out;
    out.reserve(value.size());
    for (unsigned char ch : value) {
        if (std::isalnum(ch)) out.push_back(static_cast<char>(std::tolower(ch)));
    }
    return out;
}

bool contains_token(const std::string& normalized, const std::vector<std::string>& tokens)
{
    return std::any_of(tokens.begin(), tokens.end(),
                       [&](const std::string& token) { return normalized.find(token) != std::string::npos; });
}

bool has_suffix(const std::string& normalized, const std::string& suffix)
{
    return normalized.size() >= suffix.size() &&
           normalized.compare(normalized.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::optional<double> parse_double(const std::string& value)
{
    if (value.empty()) return std::nullopt;
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || !end || *end != '\0' || !std::isfinite(parsed)) return std::nullopt;
    return parsed;
}

std::optional<int> parse_int(const std::string& value)
{
    if (const auto parsed = parse_double(value)) {
        return static_cast<int>(std::lround(*parsed));
    }
    return std::nullopt;
}

std::string extract_simpleitem_value(const std::vector<SimpleItem>& items,
                                     const std::string& wanted_name)
{
    const std::string wanted = normalize_key(wanted_name);
    for (const auto& item : items) {
        if (normalize_key(item.name) == wanted) return item.value;
    }
    return "";
}

std::string extract_topic_full(const std::string& xml)
{
    std::string out;
    if (!find_between(xml, "<wsnt:Topic", "</wsnt:Topic>", out)) return "";
    const size_t gt = out.find('>');
    if (gt == std::string::npos) return "";
    return trim(out.substr(gt + 1));
}

std::string last_token(const std::string& topic_full)
{
    const size_t slash = topic_full.find_last_of('/');
    std::string tail = (slash == std::string::npos) ? topic_full : topic_full.substr(slash + 1);
    const size_t colon = tail.find_last_of(':');
    if (colon != std::string::npos) tail = tail.substr(colon + 1);
    return tail;
}

bool is_interesting_event(const std::string& event_name)
{
    return event_name == "IvaArea" ||
           event_name == "MaskDetection" ||
           event_name == "SlipAndFallDetection" ||
           event_name == "ObjectCounterAlarm" ||
           event_name == "ObjectDetection";
}

bool is_human_like_class(const std::string& value)
{
    const std::string lower = normalize_key(value);
    return lower.find("human") != std::string::npos ||
           lower.find("person") != std::string::npos ||
           lower.find("pedestrian") != std::string::npos ||
           lower.find("body") != std::string::npos;
}

std::string classify_object_type(const std::string& class_types)
{
    std::stringstream ss(class_types);
    std::string token;
    std::string first_non_empty;
    while (std::getline(ss, token, ',')) {
        token = trim(token);
        if (token.empty()) continue;
        if (first_non_empty.empty()) first_non_empty = token;
        if (is_human_like_class(token)) return "Human";
    }
    if (is_human_like_class(first_non_empty)) return "Human";
    return first_non_empty;
}

void assign_corner_value(const std::string& normalized_name,
                         double value,
                         std::optional<double>& left,
                         std::optional<double>& top,
                         std::optional<double>& right,
                         std::optional<double>& bottom,
                         std::optional<double>& x,
                         std::optional<double>& y,
                         std::optional<double>& width,
                         std::optional<double>& height)
{
    const bool bbox_like = contains_token(normalized_name, {"bbox", "boundingbox", "rect", "rectangle", "shape"});
    if (bbox_like && has_suffix(normalized_name, "left")) left = value;
    if (bbox_like && has_suffix(normalized_name, "top")) top = value;
    if (bbox_like && has_suffix(normalized_name, "right")) right = value;
    if (bbox_like && has_suffix(normalized_name, "bottom")) bottom = value;

    if (bbox_like && (has_suffix(normalized_name, "x") || has_suffix(normalized_name, "posx"))) x = value;
    if (bbox_like && (has_suffix(normalized_name, "y") || has_suffix(normalized_name, "posy"))) y = value;
    if (bbox_like && (has_suffix(normalized_name, "width") || has_suffix(normalized_name, "w"))) width = value;
    if (bbox_like && (has_suffix(normalized_name, "height") || has_suffix(normalized_name, "h"))) height = value;

    if (normalized_name == "bboxleft") left = value;
    if (normalized_name == "bboxtop") top = value;
    if (normalized_name == "bboxright") right = value;
    if (normalized_name == "bboxbottom") bottom = value;
    if (normalized_name == "boundingboxleft") left = value;
    if (normalized_name == "boundingboxtop") top = value;
    if (normalized_name == "boundingboxright") right = value;
    if (normalized_name == "boundingboxbottom") bottom = value;
    if (normalized_name == "bboxx") x = value;
    if (normalized_name == "bboxy") y = value;
    if (normalized_name == "bboxwidth") width = value;
    if (normalized_name == "bboxheight") height = value;
}

bool try_extract_bbox_from_attr_fragment(const std::string& fragment, BoundingBox& bbox)
{
    static const std::regex re("([A-Za-z_:][-A-Za-z0-9_:.]*)=\"([^\"]*)\"");
    std::optional<double> left, top, right, bottom, x, y, width, height;
    for (std::sregex_iterator it(fragment.begin(), fragment.end(), re), end; it != end; ++it) {
        const std::string name = normalize_key((*it)[1].str());
        const auto value = parse_double((*it)[2].str());
        if (!value) continue;
        assign_corner_value(name, *value, left, top, right, bottom, x, y, width, height);
        if (!contains_token(name, {"bbox", "boundingbox", "rect", "rectangle"})) {
            if (name == "left") left = *value;
            if (name == "top") top = *value;
            if (name == "right") right = *value;
            if (name == "bottom") bottom = *value;
            if (name == "x") x = *value;
            if (name == "y") y = *value;
            if (name == "width" || name == "w") width = *value;
            if (name == "height" || name == "h") height = *value;
        }
    }

    if (left && top && right && bottom) {
        bbox = BoundingBox{*left, *top, *right, *bottom};
        return bbox.valid();
    }
    if (x && y && width && height) {
        bbox = BoundingBox{*x, *y, *x + *width, *y + *height};
        return bbox.valid();
    }
    return false;
}

bool try_extract_bbox(const std::string& xml, const std::vector<SimpleItem>& items, BoundingBox& bbox)
{
    std::optional<double> left, top, right, bottom, x, y, width, height;
    for (const auto& item : items) {
        const auto numeric = parse_double(item.value);
        if (!numeric) continue;
        assign_corner_value(normalize_key(item.name), *numeric,
                            left, top, right, bottom, x, y, width, height);
    }

    if (left && top && right && bottom) {
        bbox = BoundingBox{*left, *top, *right, *bottom};
        if (bbox.valid()) return true;
    }

    if (x && y && width && height) {
        bbox = BoundingBox{*x, *y, *x + *width, *y + *height};
        if (bbox.valid()) return true;
    }

    static const std::regex bbox_tag_re(
        R"(<[^>]*(BoundingBox|BBox|Rectangle|Rect)[^>]*>)",
        std::regex_constants::icase);
    for (std::sregex_iterator it(xml.begin(), xml.end(), bbox_tag_re), end; it != end; ++it) {
        if (try_extract_bbox_from_attr_fragment(it->str(), bbox)) return true;
    }

    return false;
}

void maybe_extract_frame_size(const std::vector<SimpleItem>& items, int& frame_w, int& frame_h)
{
    for (const auto& item : items) {
        const auto numeric = parse_int(item.value);
        if (!numeric || *numeric <= 0) continue;
        const std::string key = normalize_key(item.name);
        if (frame_w <= 0 &&
            (key == "framew" || key == "framewidth" || key == "sourcew" || key == "sourcewidth" ||
             key == "imagew" || key == "imagewidth" || key == "videow" || key == "videowidth")) {
            frame_w = *numeric;
        }
        if (frame_h <= 0 &&
            (key == "frameh" || key == "frameheight" || key == "sourceh" || key == "sourceheight" ||
             key == "imageh" || key == "imageheight" || key == "videoh" || key == "videoheight")) {
            frame_h = *numeric;
        }
    }
}

void scale_bbox_if_needed(BoundingBox& bbox,
                          int source_w,
                          int source_h,
                          int target_w,
                          int target_h,
                          int& out_frame_w,
                          int& out_frame_h)
{
    if (!bbox.valid()) {
        out_frame_w = target_w;
        out_frame_h = target_h;
        return;
    }

    if (source_w > 0 && source_h > 0 && target_w > 0 && target_h > 0 &&
        (source_w != target_w || source_h != target_h)) {
        const double sx = static_cast<double>(target_w) / static_cast<double>(source_w);
        const double sy = static_cast<double>(target_h) / static_cast<double>(source_h);
        bbox.left *= sx;
        bbox.right *= sx;
        bbox.top *= sy;
        bbox.bottom *= sy;
        out_frame_w = target_w;
        out_frame_h = target_h;
        return;
    }

    if (source_w > 0 && source_h > 0) {
        out_frame_w = source_w;
        out_frame_h = source_h;
        return;
    }

    if (target_w > 0 && target_h > 0) {
        out_frame_w = target_w;
        out_frame_h = target_h;
        return;
    }

    out_frame_w = std::max(1, static_cast<int>(std::ceil(bbox.right + 1.0)));
    out_frame_h = std::max(1, static_cast<int>(std::ceil(bbox.bottom + 1.0)));
}

std::string make_synthetic_object_id(const std::string& cam_id, const BoundingBox& bbox)
{
    const double cx = (bbox.left + bbox.right) * 0.5;
    const double h = bbox.bottom - bbox.top;
    const long long qx = static_cast<long long>(std::llround(cx / kSyntheticIdQuantPx));
    const long long qy = static_cast<long long>(std::llround(bbox.bottom / kSyntheticIdQuantPx));
    const long long qh = static_cast<long long>(std::llround(h / kSyntheticIdQuantPx));
    std::ostringstream oss;
    oss << "auto_" << cam_id << "_" << qx << "_" << qy << "_" << qh;
    return oss.str();
}

bool parse_one_xml(const std::string& xml, ParsedEv& out, const RuntimeConfig& cfg)
{
    out.topic_full = extract_topic_full(xml);
    if (out.topic_full.empty()) return false;

    const std::vector<SimpleItem> items = extract_simple_items(xml);
    find_attr_value(xml, "UtcTime=\"", out.utc);
    find_attr_value(xml, "PropertyOperation=\"", out.property_operation);

    out.rule = extract_simpleitem_value(items, "RuleName");
    out.state_str = extract_simpleitem_value(items, "State");
    out.object_id = extract_simpleitem_value(items, "ObjectId");
    out.action = extract_simpleitem_value(items, "Action");
    out.class_types = extract_simpleitem_value(items, "ClassTypes");
    out.topic = last_token(out.topic_full);
    out.object_type = classify_object_type(out.class_types);

    out.has_bbox = try_extract_bbox(xml, items, out.bbox);
    maybe_extract_frame_size(items, out.frame_w, out.frame_h);
    if (out.has_bbox) {
        scale_bbox_if_needed(out.bbox, out.frame_w, out.frame_h,
                             cfg.target_frame_w, cfg.target_frame_h,
                             out.frame_w, out.frame_h);
        if (out.object_type.empty() && out.topic == "ObjectDetection") {
            out.object_type = "Human";
        }
        if (out.object_id.empty()) {
            out.object_id = make_synthetic_object_id(cfg.cam_id, out.bbox);
        }
    }

    if (out.action.empty() && out.topic == "ObjectDetection" && !out.object_type.empty()) {
        out.action = (out.state_str == "true") ? (out.object_type + "Detected") : (out.object_type + "Cleared");
    }

    return true;
}

bool should_publish_event(const ParsedEv& ev)
{
    if (!is_interesting_event(ev.topic)) return false;
    if (ev.property_operation == "Initialized" && ev.state_str != "true") return false;
    if (ev.topic != "ObjectDetection") return true;
    return ev.state_str == "true" && ev.has_bbox && ev.bbox.valid() &&
           is_human_like_class(ev.object_type);
}

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
        th_ = std::thread([this] { loop(); });
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
                const auto now = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

                for (auto& p : fs::recursive_directory_iterator(clip_root_dir_, ec)) {
                    if (ec) break;
                    if (!p.is_regular_file(ec)) continue;
                    if (p.path().extension() != ".mp4") continue;
                    const auto mt = file_mtime_epoch_sec(p.path().string());
                    if (!mt.has_value()) continue;
                    if (now - *mt >= ttl_sec_) remove_file(p.path().string());
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

bool capture_clip_mp4_5s(const RuntimeConfig& cfg, const std::string& out_path_mp4)
{
    const std::string tmp_path = out_path_mp4 + ".tmp.mp4";
    {
        std::error_code ec;
        if (fs::exists(tmp_path, ec)) fs::remove(tmp_path, ec);
    }

    const auto exists_and_big_enough = [](const std::string& path) -> bool {
        std::error_code ec;
        if (!fs::exists(path, ec) || !fs::is_regular_file(path, ec)) return false;
        const auto sz = fs::file_size(path, ec);
        return !ec && sz > 1024;
    };

    const auto finalize = [&]() -> bool {
        if (!exists_and_big_enough(tmp_path)) return false;
        std::error_code ec;
        if (fs::exists(out_path_mp4, ec)) fs::remove(out_path_mp4, ec);
        fs::rename(tmp_path, out_path_mp4, ec);
        if (ec) {
            std::error_code remove_ec;
            fs::remove(tmp_path, remove_ec);
            return false;
        }
        return true;
    };

    {
        std::ostringstream cmd;
        cmd << "ffmpeg -y -hide_banner -loglevel error "
            << "-rtsp_transport tcp "
            << "-i \"" << cfg.rtsp_url << "\" "
            << "-t " << cfg.clip_sec << " "
            << "-an -c copy "
            << "-movflags +faststart "
            << "-reset_timestamps 1 "
            << "\"" << tmp_path << "\"";
        const int ret = std::system(cmd.str().c_str());
        if (ret == 0 && finalize()) return true;
        std::error_code ec;
        if (fs::exists(tmp_path, ec)) fs::remove(tmp_path, ec);
    }

    {
        std::ostringstream cmd;
        cmd << "ffmpeg -y -hide_banner -loglevel error "
            << "-rtsp_transport tcp "
            << "-i \"" << cfg.rtsp_url << "\" "
            << "-t " << cfg.clip_sec << " "
            << "-an -c:v libx264 -preset veryfast "
            << "-movflags +faststart "
            << "-reset_timestamps 1 "
            << "\"" << tmp_path << "\"";
        const int ret = std::system(cmd.str().c_str());
        if (ret == 0 && finalize()) return true;
        std::error_code ec;
        if (fs::exists(tmp_path, ec)) fs::remove(tmp_path, ec);
    }

    return false;
}

std::string make_event_id(const RuntimeConfig& cfg, const ParsedEv& ev)
{
    std::string utc = ev.utc;
    for (char& c : utc) {
        if (c == ':' || c == '/' || c == ' ') c = '_';
    }
    std::string oid = ev.object_id.empty() ? "noobj" : ev.object_id;
    std::string id = cfg.cam_id + "_" + ev.topic + "_" + utc + "_" + oid;

    std::string safe;
    safe.reserve(id.size());
    for (char c : id) {
        if ((c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '_' || c == '-' || c == '.') {
            safe.push_back(c);
        }
    }
    if (safe.size() > 120) safe.resize(120);
    if (!is_safe_id(safe)) return "event_invalid";
    return safe;
}

void publish_mqtt(mosquitto* mosq, const std::string& payload)
{
    mosquitto_publish(mosq, nullptr, kMqttTopic,
                      static_cast<int>(payload.size()), payload.c_str(),
                      0, false);
    mosquitto_loop(mosq, 0, 1);
}

RuntimeConfig parse_args(int argc, char** argv)
{
    RuntimeConfig cfg;
    if (argc > 1) cfg.rtsp_url = argv[1];
    if (argc > 2) cfg.mqtt_host = argv[2];
    if (argc > 3) cfg.mqtt_port = std::max(1, std::atoi(argv[3]));
    if (argc > 4) cfg.cam_id = argv[4];
    if (argc > 5) cfg.target_frame_w = std::max(1, std::atoi(argv[5]));
    if (argc > 6) cfg.target_frame_h = std::max(1, std::atoi(argv[6]));
    cfg.clip_dir = cfg.root_dir + "/clips";
    return cfg;
}
} // namespace

int main(int argc, char** argv)
{
    const RuntimeConfig cfg = parse_args(argc, argv);
    ensure_dir(cfg.root_dir);
    ensure_dir(cfg.clip_dir);

    ClipGC clip_gc(cfg.clip_dir, cfg.clip_ttl_sec, cfg.gc_interval_ms);
    clip_gc.start();

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new(nullptr, true, nullptr);
    if (!mosq) {
        std::cerr << "mosquitto_new failed\n";
        return 1;
    }
    if (mosquitto_connect(mosq, cfg.mqtt_host.c_str(), cfg.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "mosquitto_connect failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    std::string cmd =
        "ffmpeg -loglevel error -rtsp_transport tcp "
        "-i \"" + cfg.rtsp_url + "\" "
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
        const size_t n = std::fread(readbuf, 1, sizeof(readbuf), fp);
        if (n == 0) break;
        buf.append(readbuf, n);

        while (true) {
            const size_t p1 = buf.find("<?xml");
            if (p1 == std::string::npos) break;
            const size_t p2 = buf.find("<?xml", p1 + 5);
            if (p2 == std::string::npos) break;

            const std::string one = buf.substr(p1, p2 - p1);
            buf.erase(0, p2);

            ParsedEv ev{};
            if (!parse_one_xml(one, ev, cfg)) continue;
            if (ev.topic == "IvaArea") {
                std::fprintf(stderr,
                             "\n[RTSP_META][IvaArea XML]\n"
                             "topic_full=%s\n"
                             "utc=%s\n"
                             "rule=%s\n"
                             "object_id=%s\n"
                             "state=%s\n"
                             "%s\n",
                             ev.topic_full.c_str(),
                             ev.utc.c_str(),
                             ev.rule.c_str(),
                             ev.object_id.c_str(),
                             ev.state_str.c_str(),
                             one.c_str());
                std::fflush(stderr);
            }
            if (!should_publish_event(ev)) continue;

            const bool state = (ev.state_str == "true");
            std::string clip_url;
            if (state && ev.topic != "ObjectDetection") {
                const std::string event_id = make_event_id(cfg, ev);
                const std::string date_dir = cfg.clip_dir + "/" + today_ymd_local();
                ensure_dir(date_dir);
                const std::string out_mp4 = date_dir + "/" + event_id + ".mp4";
                if (capture_clip_mp4_5s(cfg, out_mp4)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(120));
                    std::error_code ec;
                    const auto sz = fs::file_size(out_mp4, ec);
                    if (!ec && sz > 1024) {
                        clip_url = "http://" + cfg.server_ip + ":" + std::to_string(cfg.http_port) +
                                   "/clips/" + today_ymd_local() + "/" + event_id + ".mp4";
                    }
                }
            }

            std::ostringstream j;
            j << "{"
              << "\"src\":\"cctv\","
              << "\"cam\":\"" << json_escape(cfg.cam_id) << "\","
              << "\"topic\":\"" << json_escape(ev.topic) << "\","
              << "\"topic_full\":\"" << json_escape(ev.topic_full) << "\","
              << "\"utc\":\"" << json_escape(ev.utc) << "\","
              << "\"rule\":\"" << json_escape(ev.rule) << "\","
              << "\"action\":\"" << json_escape(ev.action) << "\","
              << "\"objectId\":\"" << json_escape(ev.object_id) << "\","
              << "\"state\":" << (state ? "true" : "false");

            if (!ev.object_type.empty()) {
                j << ",\"objectType\":\"" << json_escape(ev.object_type) << "\"";
            }
            if (ev.has_bbox && ev.bbox.valid()) {
                const int left = static_cast<int>(std::lround(ev.bbox.left));
                const int top = static_cast<int>(std::lround(ev.bbox.top));
                const int right = static_cast<int>(std::lround(ev.bbox.right));
                const int bottom = static_cast<int>(std::lround(ev.bbox.bottom));
                const double center_x = (ev.bbox.left + ev.bbox.right) * 0.5;
                const double center_y = (ev.bbox.top + ev.bbox.bottom) * 0.5;
                j << ",\"bbox_left\":" << left
                  << ",\"bbox_top\":" << top
                  << ",\"bbox_right\":" << right
                  << ",\"bbox_bottom\":" << bottom
                  << ",\"frame_w\":" << ev.frame_w
                  << ",\"frame_h\":" << ev.frame_h
                  << ",\"center_x\":" << center_x
                  << ",\"center_y\":" << center_y;
            }

            if (ev.topic != "ObjectDetection") {
                j << ",\"clip_sec\":" << cfg.clip_sec;
                if (!clip_url.empty()) {
                    j << ",\"clip_url\":\"" << json_escape(clip_url) << "\"";
                }
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
