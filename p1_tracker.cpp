#include <iostream>
  #include <chrono>
  #include <string>
  #include <cmath>
  #include <thread>
  #include <cstdlib>
  #include <vector>
  #include <mutex>
  #include <atomic>
  #include <sstream>

  #include <opencv2/opencv.hpp>
  #include <opencv2/aruco.hpp>

  #include <mosquitto.h>

  // ---------- utils ----------
  static bool loadHomography(const std::string& path, cv::Mat& H_img2world)
  {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      if (!fs.isOpened()) return false;

      fs["H_img2world"] >> H_img2world;
      fs.release();

      return (!H_img2world.empty() && H_img2world.rows == 3 && H_img2world.cols == 3);
  }

  static cv::Point2f applyHomography(const cv::Mat& H, const cv::Point2f& p)
  {
      std::vector<cv::Point2f> src{p}, dst;
      cv::perspectiveTransform(src, dst, H);
      return dst[0];
  }

  static double calcYawWorld(const cv::Mat& H_img2world,
                             const cv::Point2f& centerPx,
                             const std::vector<cv::Point2f>& markerCorners)
  {
      cv::Point2f dir = markerCorners[1] - markerCorners[0];
      const float norm = std::sqrt(dir.x * dir.x + dir.y * dir.y);
      if (norm < 1e-6f) return 0.0;
      dir *= (1.0f / norm);

      const cv::Point2f headPx = centerPx + dir * 30.0f;
      const cv::Point2f w0 = applyHomography(H_img2world, centerPx);
      const cv::Point2f w1 = applyHomography(H_img2world, headPx);
      return std::atan2((double)(w1.y - w0.y), (double)(w1.x - w0.x));
  }

  static double normalizeAngle(double a)
  {
      constexpr double PI = 3.14159265358979323846;
      while (a > PI) a -= 2.0 * PI;
      while (a < -PI) a += 2.0 * PI;
      return a;
  }

  static int yawPriorityForMarkerId(int id)
  {
      // User-defined cube layout:
      // top=30, front=27, left=28, back=29, right=26
      switch (id) {
          case 24: return 0; // front 27
          case 21: return 1; // top 30
          case 23: return 2; // right 26
          case 25: return 3; // left 28
          case 22: return 4; // back 29
          default: return 9;
      }
  }

  static double yawOffsetForMarkerId(int id)
  {
      // Convert observed marker yaw to front(27)-referenced yaw.
      constexpr double PI = 3.14159265358979323846;
      switch (id) {
          case 27: return 0.0;       // front
          case 30: return 0.0;       // top (assumed aligned with front)
          case 26: return PI * 0.5;  // right -> front
          case 28: return -PI * 0.5; // left -> front
          case 29: return PI;        // back -> front
          default: return 0.0;
      }
  }

  static long long nowMs()
  {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()
             ).count();
  }

  static bool mqttPublishJson(mosquitto* mosq,
                              const std::string& topic,
                              const std::string& json)
  {
      int rc = mosquitto_publish(
          mosq, nullptr, topic.c_str(),
          (int)json.size(), json.c_str(),
          1, false
      );
      if (rc != MOSQ_ERR_SUCCESS) {
          std::cerr << "[WARN] mqtt publish failed rc=" << rc
                    << " (" << mosquitto_strerror(rc) << ")\n";
      }
      return (rc == MOSQ_ERR_SUCCESS);
  }

  static bool openRtspCapture(const std::string& rtspUrl, cv::VideoCapture& cap)
  {
      const std::string gst =
          "rtspsrc location=" + rtspUrl +
          " protocols=tcp latency=200 drop-on-latency=true do-rtsp-keep-alive=true tcp-timeout=5000000 ! "
          "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! queue leaky=downstream max-size-buffers=1 ! "
          "appsink sync=false max-buffers=1 drop=true";

      if (cap.open(gst, cv::CAP_GSTREAMER)) return true;

      setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
             "rtsp_transport;tcp|fflags;nobuffer|max_delay;500000|stimeout;5000000", 1);
      if (cap.open(rtspUrl, cv::CAP_FFMPEG)) {
          cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
          return true;
      }
      return false;
  }

  // ---------- homography subscribe ----------
  struct SharedHomography {
      std::mutex mtx;
      cv::Mat H_img2world;
      std::atomic<long long> updateCount{0};
  };

  static bool parseHomographyJson(const std::string& json, cv::Mat& H_out)
  {
      if (json.find("\"type\"") == std::string::npos ||
          json.find("homography") == std::string::npos ||
          json.find("H_img2world") == std::string::npos) {
          return false;
      }

      const size_t dataPos = json.find("\"data\"");
      if (dataPos == std::string::npos) return false;

      const size_t lb = json.find('[', dataPos);
      const size_t rb = json.find(']', lb == std::string::npos ? dataPos : lb);
      if (lb == std::string::npos || rb == std::string::npos || rb <= lb) return false;

      std::string nums = json.substr(lb + 1, rb - lb - 1);
      for (char& c : nums) if (c == ',') c = ' ';

      std::stringstream ss(nums);
      std::vector<double> v;
      double x = 0.0;
      while (ss >> x) v.push_back(x);

      if (v.size() != 9) return false;

      H_out = (cv::Mat_<double>(3, 3) <<
          v[0], v[1], v[2],
          v[3], v[4], v[5],
          v[6], v[7], v[8]);

      return true;
  }

  static void on_message_cb(struct mosquitto*,
                            void* userdata,
                            const struct mosquitto_message* msg)
  {
      if (!userdata || !msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;

      const std::string topic(msg->topic);
      if (topic != "wiserisk/map/H_img2world") return;

      const std::string payload((const char*)msg->payload, (size_t)msg->payloadlen);
      cv::Mat newH;
      if (!parseHomographyJson(payload, newH)) {
          std::cerr << "[WARN] homography payload parse failed\n";
          return;
      }

      auto* shared = static_cast<SharedHomography*>(userdata);
      {
          std::lock_guard<std::mutex> lk(shared->mtx);
          shared->H_img2world = newH.clone();
          shared->updateCount.fetch_add(1);
      }
      std::cout << "[INFO] H_img2world updated from MQTT (wiserisk/map/H_img2world)\n";
  }

  // ---------- main ----------
  int main(int argc, char** argv)
  {
      const std::string rtspUrl = (argc > 1)
          ? argv[1]
          : "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
      const int targetMarkerMinId = 26;
      const int targetMarkerMaxId = 30;
      const std::string mqttHost = (argc > 2) ? argv[2] : "192.168.100.10";
      const int mqttPort = (argc > 3) ? std::stoi(argv[3]) : 1883;
      const std::string mqttTopic = (argc > 4) ? argv[4] : "wiserisk/p1/pose";
      const std::string homographyYaml = (argc > 5)
          ? argv[5]
          : "/home/pi/final_veda_test/map/build/H_img2world.yaml";

      SharedHomography shared;
      if (!loadHomography(homographyYaml, shared.H_img2world)) {
          std::cerr << "[ERR] failed to load homography: " << homographyYaml << "\n";
          return 1;
      }

      cv::VideoCapture cap;
      if (!openRtspCapture(rtspUrl, cap)) {
          std::cerr << "[ERR] RTSP open failed\n";
          return 2;
      }

      auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
      cv::aruco::DetectorParameters params;
      cv::aruco::ArucoDetector detector(dict, params);

      mosquitto_lib_init();
      mosquitto* mosq = mosquitto_new(nullptr, true, &shared);
      if (!mosq) {
          std::cerr << "[ERR] mosquitto_new failed\n";
          return 3;
      }

      mosquitto_reconnect_delay_set(mosq, 1, 10, true);
      mosquitto_message_callback_set(mosq, on_message_cb);
      
      if (mosquitto_connect(mosq, mqttHost.c_str(), mqttPort, 60) != MOSQ_ERR_SUCCESS) {
          std::cerr << "[ERR] mosquitto_connect failed: " << mqttHost << ":" << mqttPort << "\n";
          mosquitto_destroy(mosq);
          mosquitto_lib_cleanup();
          return 4;
      }

      if (mosquitto_subscribe(mosq, nullptr, "wiserisk/map/H_img2world", 1) != MOSQ_ERR_SUCCESS) {
          std::cerr << "[ERR] subscribe failed: wiserisk/map/H_img2world\n";
          mosquitto_disconnect(mosq);
          mosquitto_destroy(mosq);
          mosquitto_lib_cleanup();
          return 5;
      }

      if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS) {
          std::cerr << "[ERR] mosquitto_loop_start failed\n";
          mosquitto_disconnect(mosq);
          mosquitto_destroy(mosq);
          mosquitto_lib_cleanup();
          return 6;
      }

      std::cout << "[OK] start tracking, H_yaml=" << homographyYaml
                << ", mqtt sub=wiserisk/map/H_img2world, pub=" << mqttTopic << "\n";

      int readFailCount = 0;
      long long frameCount = 0;
      long long detectCount = 0;
      long long publishCount = 0;
      auto lastStatTs = std::chrono::steady_clock::now();

      while (true) {
          cv::Mat frame;
          if (!cap.read(frame) || frame.empty()) {
              readFailCount++;
              if (readFailCount % 20 == 0) {
                  std::cerr << "[WARN] frame read failed x" << readFailCount << " (reconnecting)\n";
                  cap.release();
                  std::this_thread::sleep_for(std::chrono::milliseconds(300));
                  if (!openRtspCapture(rtspUrl, cap)) {
                      std::cerr << "[WARN] RTSP reconnect failed\n";
                  }
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(30));
              continue;
          }

          readFailCount = 0;
          frameCount++;

          cv::Mat H_now;
          {
              std::lock_guard<std::mutex> lk(shared.mtx);
              H_now = shared.H_img2world.clone();
          }

          std::vector<std::vector<cv::Point2f>> corners;
          std::vector<int> ids;
          detector.detectMarkers(frame, corners, ids);

          cv::Point2f sumCenter(0.0f, 0.0f);
          int matchedCount = 0;
          size_t yawIndex = 0;
          int yawMarkerId = -1;
          double bestArea = -1.0;
          int bestPriority = 99;

          for (size_t i = 0; i < ids.size(); i++) {
              if (ids[i] < targetMarkerMinId || ids[i] > targetMarkerMaxId) continue;

              cv::Point2f c(0, 0);
              for (const auto& p : corners[i]) c += p;
              c *= (1.0f / 4.0f);

              sumCenter += c;
              matchedCount++;

              const int pri = yawPriorityForMarkerId(ids[i]);
              const double area = cv::contourArea(corners[i]);
              if (pri < bestPriority || (pri == bestPriority && area > bestArea)) {
                  bestPriority = pri;
                  bestArea = area;
                  yawIndex = i;
                  yawMarkerId = ids[i];
              }
          }

          if (matchedCount > 0) {
              const cv::Point2f centerPx = sumCenter * (1.0f / matchedCount);
              const cv::Point2f w = applyHomography(H_now, centerPx);
              const double yawRaw = calcYawWorld(H_now, centerPx, corners[yawIndex]);
              const double yaw = normalizeAngle(yawRaw + yawOffsetForMarkerId(yawMarkerId));
              const long long ts = nowMs();

              char buf[256];
              std::snprintf(buf, sizeof(buf),
                            "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts_ms\":%lld}",
                            w.x, w.y, yaw, ts);

              if (mqttPublishJson(mosq, mqttTopic, buf)) {
                  detectCount++;
                  publishCount++;
              }
          }

          const auto nowSteady = std::chrono::steady_clock::now();
          const auto elapsedMs =
              std::chrono::duration_cast<std::chrono::milliseconds>(nowSteady - lastStatTs).count();
          if (elapsedMs >= 1000) {
              const double fps = (elapsedMs > 0) ? (frameCount * 1000.0 / elapsedMs) : 0.0;
              std::cout << "[STAT] fps=" << fps
                        << " ids_seen=" << ids.size()
                        << " id26_30_detect_total=" << detectCount
                        << " pose_pub_total=" << publishCount
                        << " H_updates=" << shared.updateCount.load()
                        << " read_fail_streak=" << readFailCount
                        << "\n";
              frameCount = 0;
              lastStatTs = nowSteady;
          }
      }

      mosquitto_loop_stop(mosq, true);
      mosquitto_disconnect(mosq);
      mosquitto_destroy(mosq);
      mosquitto_lib_cleanup();
      return 0;
  }
