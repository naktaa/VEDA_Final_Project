#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <map>
#include <vector>
#include <string>

// ---------- 설정: 4개 마커 ID와 월드좌표(m) ----------
struct WorldPt { double x,y; };

int main(int argc, char** argv)
{
    // RTSP
    const std::string rtsp =
        "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";

    // 출력 파일
    std::string outYaml = "H_img2world.yaml";
    if (argc >= 2) outYaml = argv[1]; // 예: ./calib ../H_img2world.yaml

    // ✅ 여기 ID 4개를 "바닥에 붙인 마커"로 맞춰서 사용
    // 예시: ID 10,11,12,13을 바닥 네 꼭짓점에 둔다.
    std::map<int, WorldPt> id2world = {
        {10, {1.0, 330.0}},
        {11, {95.0, 330.0}},
        {12, {1.0, 1.0}},
        {13, {95.0, 1.0}},
    };

    // ---- open RTSP ----
    cv::VideoCapture cap(rtsp);
    if (!cap.isOpened()) {
        std::cerr << "[ERR] RTSP open failed\n";
        return 1;
    }

    // ---- aruco ----
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    // Detection tuning for 170mm markers at 1024x768
    params.adaptiveThreshWinSizeMin = 3;
    params.adaptiveThreshWinSizeMax = 53;
    params.adaptiveThreshWinSizeStep = 4;
    params.minMarkerPerimeterRate = 0.02;
    params.maxMarkerPerimeterRate = 4.0;
    params.polygonalApproxAccuracyRate = 0.03;
    params.minCornerDistanceRate = 0.05;
    params.minDistanceToBorder = 3;
    params.minMarkerDistanceRate = 0.05;
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params.cornerRefinementWinSize = 5;
    params.cornerRefinementMaxIterations = 50;
    params.cornerRefinementMinAccuracy = 0.05;
    params.perspectiveRemovePixelPerCell = 8;
    params.perspectiveRemoveIgnoredMarginPerCell = 0.13;
    params.maxErroneousBitsInBorderRate = 0.35;
    params.errorCorrectionRate = 0.6;
    cv::aruco::ArucoDetector detector(dict, params);

    std::cout << "[INFO] Waiting for 4 markers (";
    for (auto& kv : id2world) std::cout << kv.first << " ";
    std::cout << ")...\n";
    std::cout << "[INFO] Press 's' to save when all are detected, 'q' to quit.\n";

    // 픽셀 중심 저장
    std::map<int, cv::Point2f> id2img;

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) continue;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(frame, corners, ids);

        id2img.clear();

        // 검출된 마커들의 중심 계산
        for (size_t i=0;i<ids.size();i++){
            int id = ids[i];
            if (!id2world.count(id)) continue;

            cv::Point2f c(0,0);
            for (auto& p: corners[i]) c += p;
            c *= 0.25f;
            id2img[id] = c;

            // 화면 표시
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            cv::circle(frame, c, 6, cv::Scalar(0,255,0), -1);
            cv::putText(frame, "ID "+std::to_string(id),
                        c + cv::Point2f(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0,255,0), 2);
        }

        // 상태 표시
        int found = (int)id2img.size();
        cv::putText(frame, "found: " + std::to_string(found) + "/4",
                    cv::Point(20,40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0,255,255), 2);

        cv::imshow("calib", frame);

        int k = cv::waitKey(1);
        if (k == 'q' || k == 27) break;

        // 저장 시도
        if (k == 's') {
            if (id2img.size() != id2world.size()) {
                std::cerr << "[WARN] Not all 4 markers detected.\n";
                continue;
            }

            // imgPts / worldPts 쌍 만들기
            std::vector<cv::Point2f> imgPts;
            std::vector<cv::Point2f> worldPts;

            // id2world의 순서를 그대로 사용 (10,11,12,13)
            for (auto& kv : id2world) {
                int id = kv.first;
                auto wp = kv.second;
                auto ip = id2img[id];

                imgPts.push_back(ip);
                worldPts.push_back(cv::Point2f((float)wp.x, (float)wp.y));
            }

            // Homography 계산: image -> world
            cv::Mat H = cv::findHomography(imgPts, worldPts, cv::RANSAC, 3.0);
            if (H.empty()) {
                std::cerr << "[ERR] findHomography failed\n";
                continue;
            }
            H.convertTo(H, CV_64F);

            // YAML 저장
            cv::FileStorage fs(outYaml, cv::FileStorage::WRITE);
            fs << "H_img2world" << H;
            fs.release();

            std::cout << "[OK] Saved homography to: " << outYaml << "\n";
            std::cout << H << "\n";
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
