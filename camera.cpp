#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <mosquitto.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <ctime>

using namespace cv;

/* ===================== Utils ===================== */
static Rect clampRect(const Rect& r, int w, int h) { return r & Rect(0,0,w,h); }
static Rect expandRect(const Rect& r, int w, int h, float ratio) {
    int dx = int(r.width  * ratio);
    int dy = int(r.height * ratio);
    return clampRect(Rect(r.x-dx, r.y-dy, r.width+2*dx, r.height+2*dy), w, h);
}
static float IoU(const Rect& a, const Rect& b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width,  b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    int inter = std::max(0, x2-x1) * std::max(0, y2-y1);
    int uni = a.area() + b.area() - inter;
    return (uni > 0) ? (float)inter / (float)uni : 0.f;
}
static inline Point2f centerOf(const Rect& r){
    return Point2f(r.x + r.width*0.5f, r.y + r.height*0.5f);
}
static inline float dist_px(const Point2f& a, const Point2f& b){
    float dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
}
static inline float aspectRatioWH(const Rect& r){
    return (r.height>0) ? (float)r.width / (float)r.height : 0.f;
}
static Rect ema_rect(const Rect& prev, const Rect& cur, float a){
    int x = (int)std::lround((1-a)*prev.x + a*cur.x);
    int y = (int)std::lround((1-a)*prev.y + a*cur.y);
    int w = (int)std::lround((1-a)*prev.width + a*cur.width);
    int h = (int)std::lround((1-a)*prev.height + a*cur.height);
    return Rect(x,y,w,h);
}

/* ===================== Privacy Blur ===================== */
static void apply_privacy_blur(Mat& bgr, const std::vector<Rect>& rects) {
    for (const auto& rr : rects) {
        Rect r = clampRect(rr, bgr.cols, bgr.rows);
        if (r.area() <= 0) continue;

        Mat roi = bgr(r);

        int kx = std::max(21, (r.width  / 7) | 1);
        int ky = std::max(21, (r.height / 7) | 1);
        kx = std::min(kx, 99);
        ky = std::min(ky, 99);

        GaussianBlur(roi, roi, Size(kx, ky), 0);
    }
}

static bool valid_blur_rect(const Rect& r, int W, int H){
    if(r.area() <= 0) return false;
    float areaRatio = (float)r.area() / (float)(W*H);
    if(areaRatio > 0.45f) return false;          // 전체 블러 방지
    float ar = aspectRatioWH(r);
    if(ar < 0.15f || ar > 4.0f) return false;
    if(r.width < 18 || r.height < 30) return false;
    return true;
}

static void merge_rects(std::vector<Rect>& rs, float iou_th=0.60f){
    std::vector<Rect> out;
    out.reserve(rs.size());
    for(const auto& r: rs){
        bool merged=false;
        for(auto& o: out){
            if(IoU(r,o) > iou_th){
                o = o | r;
                merged=true; break;
            }
        }
        if(!merged) out.push_back(r);
    }
    rs.swap(out);
}

/* ===================== Zones (옵션: 파일로 불러오거나 고정값 써도 됨) ===================== */
enum class ZoneType { BED, FLOOR };
struct Zone { ZoneType type; std::string name; Rect roi; };
static const char* ztype_str(ZoneType t){ return (t==ZoneType::BED) ? "BED" : "FLOOR"; }

static bool load_zones(const std::string& path, std::vector<Zone>& zones){
    std::ifstream fin(path);
    if(!fin) return false;
    zones.clear();
    std::string t,name; int x,y,w,h;
    while(fin >> t >> name >> x >> y >> w >> h){
        ZoneType zt;
        if(t=="BED") zt=ZoneType::BED;
        else if(t=="FLOOR") zt=ZoneType::FLOOR;
        else continue;
        zones.push_back(Zone{zt, name, Rect(x,y,w,h)});
    }
    return !zones.empty();
}

static bool rectContainsCenter(const Rect& R, const Rect& r){
    Point2f c = centerOf(r);
    return R.contains(Point((int)c.x,(int)c.y));
}
static int find_zone_by_center(const std::vector<Zone>& zones, const Rect& person){
    for(int i=0;i<(int)zones.size();++i){
        if(zones[i].roi.area()<=0) continue;
        if(rectContainsCenter(zones[i].roi, person)) return i;
    }
    return -1;
}
static void draw_zones(Mat& img, const std::vector<Zone>& zones){
    for(auto& z: zones){
        Scalar col = (z.type==ZoneType::BED) ? Scalar(0,255,255) : Scalar(0,255,0);
        rectangle(img, z.roi, col, 2);
        putText(img, z.name + ":" + ztype_str(z.type),
                Point(z.roi.x, std::max(0, z.roi.y-6)),
                FONT_HERSHEY_SIMPLEX, 0.6, col, 2);
    }
}

/* ===================== SSD Person detect ===================== */
struct Det { Rect r; float score; };

static std::vector<Det> detect_person_tf_ssd(
    cv::dnn::Net& net,
    const cv::Mat& bgr,
    float confTh
){
    std::vector<Det> out;

    Mat blob = cv::dnn::blobFromImage(
        bgr, 1.0, Size(300,300),
        Scalar(127.5,127.5,127.5),
        true, false
    );

    net.setInput(blob);
    Mat det = net.forward();
    if (det.dims != 4 || det.size[3] != 7) return out;

    Mat detMat(det.size[2], det.size[3], CV_32F, det.ptr<float>());
    int w = bgr.cols, h = bgr.rows;

    for(int i=0;i<detMat.rows;i++){
        float image_id = detMat.at<float>(i,0);
        if(image_id < 0) continue;

        int classId = (int)detMat.at<float>(i,1);
        float score = detMat.at<float>(i,2);
        if(score < confTh) continue;
        if(classId != 1) continue;

        int left   = (int)(detMat.at<float>(i,3) * w);
        int top    = (int)(detMat.at<float>(i,4) * h);
        int right  = (int)(detMat.at<float>(i,5) * w);
        int bottom = (int)(detMat.at<float>(i,6) * h);

        Rect r(Point(left,top), Point(right,bottom));
        r = clampRect(r, w, h);
        if(r.area()<=0) continue;

        out.push_back({r, score});
    }
    return out;
}

/* ===================== Tracking ===================== */
struct Track {
    int id=-1;
    Rect r;
    float score=0.f;
    int expire_ms=0;

    Point2f prev_c;
    int prev_h=0;

    float base_h=0.f;

    bool fall_pending=false;
    int  pending_ms=0;

    int post_lie_start_ms = 0;
    int last_fall_ms=-1000000;

    float dy=0.f;
    float ar=0.f;
    float h_ratio=1.f;

    bool updated=false;
    int  last_update_ms=0;

    Rect smooth_r;
    bool has_smooth=false;

    Rect last_good_r;
    int  last_good_ms=0;
};

static void update_tracks_iou(
    std::vector<Track>& tracks,
    const std::vector<Det>& dets,
    int now_ms,
    int hold_ms,
    float iou_th,
    int& next_id
){
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
        [&](const Track& t){ return now_ms > t.expire_ms; }), tracks.end());

    for(auto& tr: tracks) tr.updated = false;

    std::vector<int> used(dets.size(), 0);

    for(auto& tr: tracks){
        int best=-1; float best_iou=0.f;
        for(int j=0;j<(int)dets.size();++j){
            if(used[j]) continue;
            float iou = IoU(tr.r, dets[j].r);
            if(iou > best_iou){
                best_iou = iou;
                best = j;
            }
        }
        if(best>=0 && best_iou >= iou_th){
            tr.r = dets[best].r;
            tr.score = dets[best].score;
            tr.expire_ms = now_ms + hold_ms;
            tr.updated = true;
            tr.last_update_ms = now_ms;
            used[best]=1;
        }
    }

    for(int j=0;j<(int)dets.size();++j){
        if(used[j]) continue;
        Track nt;
        nt.id = next_id++;
        nt.r = dets[j].r;
        nt.score = dets[j].score;
        nt.expire_ms = now_ms + hold_ms;
        nt.prev_c = centerOf(nt.r);
        nt.prev_h = nt.r.height;
        nt.base_h = (float)nt.r.height;

        nt.updated = true;
        nt.last_update_ms = now_ms;

        nt.smooth_r = nt.r;
        nt.has_smooth = true;

        nt.last_good_r = nt.r;
        nt.last_good_ms = now_ms;

        tracks.push_back(nt);
    }
}

/* ===================== Global FallLock ===================== */
struct FallEvent { Point2f pos; int t_ms; };

static void prune_falls(std::vector<FallEvent>& falls, int now_ms, int lock_ms){
    falls.erase(std::remove_if(falls.begin(), falls.end(),
        [&](const FallEvent& f){ return (now_ms - f.t_ms) > lock_ms; }), falls.end());
}

static bool is_dup_fall(const std::vector<FallEvent>& falls, const Point2f& pos, int now_ms, int lock_ms, float lock_dist){
    for(const auto& f: falls){
        if(now_ms - f.t_ms > lock_ms) continue;
        if(dist_px(pos, f.pos) < lock_dist) return true;
    }
    return false;
}

/* ===================== MQTT Helper ===================== */
static std::string iso8601_now_utc(){
    std::time_t t = std::time(nullptr);
    std::tm gm{};
    gmtime_r(&t, &gm);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
        gm.tm_year+1900, gm.tm_mon+1, gm.tm_mday, gm.tm_hour, gm.tm_min, gm.tm_sec);
    return std::string(buf);
}

static bool mqtt_publish_json(struct mosquitto* mosq,
                              const std::string& topic,
                              const std::string& payload,
                              int qos=1)
{
    int rc = mosquitto_publish(mosq, nullptr, topic.c_str(),
                              (int)payload.size(), payload.c_str(),
                              qos, false);
    return (rc == MOSQ_ERR_SUCCESS);
}

/* ===================== Main ===================== */
int main(int argc, char** argv){
    // ---- 설정(필요한 것만 바꿔서 쓰세요) ----
    const int W = 960, H = 540;
    const int FPS = 30;

    const std::string PB    = "model/frozen_inference_graph.pb";
    const std::string PBTXT = "model/ssd_mobilenet_v2_coco_2018_03_29.pbtxt";
    const std::string ZONES_PATH = "zones.txt";

    // 서버 주소(영상/ MQTT)
    const std::string SERVER_IP = "192.168.0.124";   // ★ 라즈베리파이 서버 IP로 변경
    const int VIDEO_UDP_PORT = 5600;
    const int MQTT_PORT = 1884;
    const std::string MQTT_TOPIC = "hospital/fall";

    // ---- 캡처: libcamera -> appsink (BGR) ----
    // Pi OS(Bookworm) 기준으로 libcamera-vid/appsink 파이프라인이 잘 동작합니다.
    std::ostringstream capss;
    capss
        << "libcamerasrc ! "
        << "video/x-raw,width=" << W << ",height=" << H << ",framerate=" << FPS << "/1 ! "
        << "videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1";
    VideoCapture cap(capss.str(), CAP_GSTREAMER);
    if(!cap.isOpened()){
        std::cerr << "Failed to open camera via GStreamer.\n"
                  << "Pipeline:\n" << capss.str() << std::endl;
        return 1;
    }

    // ---- 송출: appsrc -> x264enc -> rtph264pay -> udpsink ----
  std::ostringstream outss;
outss
    << "appsrc ! videoconvert ! video/x-raw,format=I420,framerate=" << FPS << "/1 ! "
    << "x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 key-int-max=" << FPS << " ! "
    << "rtph264pay config-interval=1 pt=96 ! "
    << "rtspclientsink location=rtsp://" << SERVER_IP << ":8554/live";
    VideoWriter writer(outss.str(), CAP_GSTREAMER, 0, (double)FPS, Size(W,H), true);
    if(!writer.isOpened()){
        std::cerr << "Failed to open UDP streamer via GStreamer.\n"
                  << "Pipeline:\n" << outss.str() << std::endl;
        return 1;
    }

    // ---- DNN 로드 ----
    cv::dnn::Net ssd;
    try { ssd = cv::dnn::readNetFromTensorflow(PB, PBTXT); }
    catch(const std::exception& e){
        std::cerr << "readNetFromTensorflow failed: " << e.what() << std::endl;
        return 1;
    }

    // ---- Zones 로드 ----
    std::vector<Zone> zones;
    if(!load_zones(ZONES_PATH, zones)){
        std::cerr << "zones.txt not found. ROI를 파일로 만들어두셔야 합니다: " << ZONES_PATH << std::endl;
        return 1;
    }
    for(auto& z: zones) z.roi = clampRect(z.roi, W, H);

    // ---- MQTT init/connect ----
    mosquitto_lib_init();
    struct mosquitto* mosq = mosquitto_new("fall_cam_node", true, nullptr);
    if(!mosq){
        std::cerr << "mosquitto_new failed\n";
        return 1;
    }

    // (옵션) 재연결 관련
    mosquitto_reconnect_delay_set(mosq, 1, 10, true);

    int rc = mosquitto_connect(mosq, SERVER_IP.c_str(), MQTT_PORT, 30);
    if(rc != MOSQ_ERR_SUCCESS){
        std::cerr << "mosquitto_connect failed: " << mosquitto_strerror(rc) << std::endl;
        // 연결 실패해도 영상은 계속 보내고, 추후 reconnect 시도하게 할 수도 있습니다.
    }
    mosquitto_loop_start(mosq);

    /* ===================== Tunings ===================== */
    float CONF = 0.30f;

    const int   TRACK_HOLD_MS = 15000;
    const float IOU_TH = 0.20f;

    // 블러 안정화/유지(끊김 최소)
    const int   BLUR_HOLD_MS  = 1800;
    const int   BLUR_STALE_MS = 2500;
    const float BLUR_EMA_A    = 0.25f;
    const float BLUR_EXPAND   = 0.30f;

    // 낙상
    const float DY_START_TH   = 0.05f * H;
    const float HSTART_RATIO  = 0.80f;

    const float AR_LIE_TH         = 1.15f;
    const int   PENDING_WINDOW_MS = 1200;
    const int   LIE_HOLD_MS       = 200;

    const float BASE_ALPHA    = 0.03f;
    const float AR_STAND_MAX  = 0.95f;

    const int TRACK_COOLDOWN_MS = 20000;

    const int   FALL_LOCK_MS   = 60000;
    const float FALL_LOCK_DIST = 220.f;

    bool DBG = false; // headless라면 false 권장

    /* ===================== States ===================== */
    std::vector<Track> tracks;
    int next_id = 1;

    struct BlurHold { Rect r; int expire_ms; };
    std::vector<BlurHold> blur_holds;

    std::vector<FallEvent> recent_falls;

    int frame_count = 0;
    Mat frame;

    std::cout << "[RUN] streaming to udp://" << SERVER_IP << ":" << VIDEO_UDP_PORT
              << " , mqtt " << SERVER_IP << ":" << MQTT_PORT << " topic=" << MQTT_TOPIC << std::endl;

    while(true){
        if(!cap.read(frame) || frame.empty()){
            // 카메라 순간 끊김이면 조금 기다렸다 재시도
            cv::waitKey(1);
            continue;
        }
        frame_count++;

        int t_ms = (int)std::llround((double)frame_count * 1000.0 / (double)FPS);

        prune_falls(recent_falls, t_ms, FALL_LOCK_MS);

        auto dets = detect_person_tf_ssd(ssd, frame, CONF);
        update_tracks_iou(tracks, dets, t_ms, TRACK_HOLD_MS, IOU_TH, next_id);

        Mat outFrame = frame.clone();

        // ---- (1) 블러 홀드 생성(끊김 최소 + 전체블러 방지) ----
        for(auto& tr : tracks){
            if(tr.updated){
                if(!tr.has_smooth){
                    tr.smooth_r = tr.r;
                    tr.has_smooth = true;
                }else{
                    tr.smooth_r = ema_rect(tr.smooth_r, tr.r, BLUR_EMA_A);
                }
                if(valid_blur_rect(tr.smooth_r, W, H)){
                    tr.last_good_r  = tr.smooth_r;
                    tr.last_good_ms = t_ms;
                }
            }

            bool use_hold = (t_ms - tr.last_good_ms) <= BLUR_STALE_MS;
            if(!use_hold) continue;

            Rect br = tr.last_good_r;
            if(!valid_blur_rect(br, W, H)) continue;

            br = expandRect(br, W, H, BLUR_EXPAND);
            blur_holds.push_back({br, t_ms + BLUR_HOLD_MS});
        }

        blur_holds.erase(std::remove_if(blur_holds.begin(), blur_holds.end(),
            [&](const BlurHold& b){ return t_ms > b.expire_ms; }), blur_holds.end());

        std::vector<Rect> blurRects;
        blurRects.reserve(blur_holds.size());
        for(auto& b: blur_holds) blurRects.push_back(b.r);
        merge_rects(blurRects, 0.60f);

        // ---- (2) 블러 먼저 ----
        apply_privacy_blur(outFrame, blurRects);

        // ---- (3) 낙상 판단 + MQTT publish ----
        for(auto& tr: tracks){
            int zi = find_zone_by_center(zones, tr.r);
            bool in_bed   = (zi >= 0 && zones[zi].type == ZoneType::BED);
            bool in_floor = (zi >= 0 && zones[zi].type == ZoneType::FLOOR);

            Point2f c = centerOf(tr.r);
            int h_now = tr.r.height;
            float ar  = aspectRatioWH(tr.r);

            if(ar <= AR_STAND_MAX){
                if(tr.base_h <= 0.f) tr.base_h = (float)h_now;
                tr.base_h = (1.0f - BASE_ALPHA) * tr.base_h + BASE_ALPHA * (float)h_now;
            }

            float h_ratio = (tr.base_h > 1.f) ? ((float)h_now / tr.base_h) : 1.f;
            float dy = c.y - tr.prev_c.y;

            tr.dy = dy; tr.ar = ar; tr.h_ratio = h_ratio;

            bool start_event = (dy >= DY_START_TH) || (h_ratio <= HSTART_RATIO);
            if(!tr.fall_pending && start_event){
                tr.fall_pending = true;
                tr.pending_ms = t_ms;
                tr.post_lie_start_ms = 0;
            }

            bool lie_now = (ar >= AR_LIE_TH);
            if(tr.fall_pending){
                if(t_ms - tr.pending_ms > PENDING_WINDOW_MS){
                    tr.fall_pending = false;
                    tr.post_lie_start_ms = 0;
                } else {
                    if(lie_now){
                        if(tr.post_lie_start_ms == 0) tr.post_lie_start_ms = t_ms;
                    } else {
                        tr.post_lie_start_ms = 0;
                    }

                    bool lie_hold_ok = (tr.post_lie_start_ms != 0) && ((t_ms - tr.post_lie_start_ms) >= LIE_HOLD_MS);
                    bool allow_event = (!in_bed) && (in_floor || zi < 0);

                    bool cooldown = (t_ms - tr.last_fall_ms) < TRACK_COOLDOWN_MS;
                    bool dup_lock = is_dup_fall(recent_falls, c, t_ms, FALL_LOCK_MS, FALL_LOCK_DIST);

                    if(allow_event && lie_hold_ok && !cooldown && !dup_lock){
                        tr.last_fall_ms = t_ms;
                        recent_falls.push_back({c, t_ms});

                        // ---- JSON payload 생성 ----
                        std::ostringstream js;
                        js << "{"
                           << "\"type\":\"fall\","
                           << "\"ts_utc\":\"" << iso8601_now_utc() << "\","
                           << "\"frame\":" << frame_count << ","
                           << "\"time_ms\":" << t_ms << ","
                           << "\"track_id\":" << tr.id << ","
                           << "\"score\":" << tr.score << ","
                           << "\"bbox\":{"
                               << "\"x\":" << tr.r.x << ","
                               << "\"y\":" << tr.r.y << ","
                               << "\"w\":" << tr.r.width << ","
                               << "\"h\":" << tr.r.height
                           << "},"
                           << "\"metrics\":{"
                               << "\"dy\":" << dy << ","
                               << "\"h_ratio\":" << h_ratio << ","
                               << "\"ar\":" << ar
                           << "},"
                           << "\"zone\":{"
                               << "\"idx\":" << zi << ","
                               << "\"in_bed\":" << (in_bed?1:0) << ","
                               << "\"in_floor\":" << (in_floor?1:0)
                           << "}"
                           << "}";

                        std::string payload = js.str();

                        // ---- MQTT publish ----
                        mqtt_publish_json(mosq, MQTT_TOPIC, payload, 1);

                        std::cout << "[MQTT FALL] " << payload << std::endl;
                    }
                }
            }

            tr.prev_c = c;
            tr.prev_h = h_now;
        }

        // ---- (4) 영상 송출(계속) ----
        writer.write(outFrame);

        // headless라면 imshow/키입력 제거 권장
        if(DBG){
            imshow("preview", outFrame);
            int k = waitKey(1);
            if(k=='q' || k==27) break;
        }
    }

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}
