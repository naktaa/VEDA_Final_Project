#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

static bool hasImageExt(const std::string& path) {
    auto pos = path.find_last_of('.');
    if (pos == std::string::npos) return false;
    std::string ext = path.substr(pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    return (ext == "jpg" || ext == "jpeg" || ext == "png" || ext == "bmp");
}

static void printUsage(const char* exe) {
    std::cout << "Usage: " << exe
              << " <image_dir> [board_w] [board_h] [square_size] [out_yaml]\n"
              << "  board_w/board_h: internal corners (9x7 squares -> 8x6 corners)\n"
              << "  square_size: real size of one square (e.g., 3.0 for 30mm if using cm)\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string imageDir = argv[1];
    const int boardW = (argc > 2) ? std::stoi(argv[2]) : 8;
    const int boardH = (argc > 3) ? std::stoi(argv[3]) : 6;
    const double squareSize = (argc > 4) ? std::stod(argv[4]) : 3.0;
    const std::string outYaml = (argc > 5) ? argv[5] : "config/camera.yaml";

    if (!std::filesystem::exists(imageDir)) {
        std::cerr << "[ERR] image_dir not found: " << imageDir << "\n";
        return 2;
    }

    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(imageDir)) {
        if (!entry.is_regular_file()) continue;
        const std::string p = entry.path().string();
        if (hasImageExt(p)) files.push_back(p);
    }
    std::sort(files.begin(), files.end());

    if (files.empty()) {
        std::cerr << "[ERR] no images in: " << imageDir << "\n";
        return 3;
    }

    const cv::Size pattern(boardW, boardH);
    std::vector<cv::Point3f> objp;
    objp.reserve((size_t)boardW * (size_t)boardH);
    for (int y = 0; y < boardH; ++y) {
        for (int x = 0; x < boardW; ++x) {
            objp.emplace_back((float)(x * squareSize),
                              (float)(y * squareSize),
                              0.0f);
        }
    }

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;
    std::vector<std::string> failed;
    int used = 0;
    cv::Size imageSize;

    for (const auto& fn : files) {
        cv::Mat img = cv::imread(fn);
        if (img.empty()) {
            failed.push_back(std::filesystem::path(fn).filename().string());
            continue;
        }
        if (imageSize.width == 0) {
            imageSize = img.size();
        } else if (img.size() != imageSize) {
            std::cerr << "[WARN] size mismatch, skip: " << fn << "\n";
            failed.push_back(std::filesystem::path(fn).filename().string());
            continue;
        }

        cv::Mat gray;
        if (img.channels() == 3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = img;
        }

        std::vector<cv::Point2f> corners;
        const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
        const bool ok = cv::findChessboardCorners(gray, pattern, corners, flags);
        if (!ok) {
            failed.push_back(std::filesystem::path(fn).filename().string());
            continue;
        }

        cv::cornerSubPix(
            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001)
        );

        objpoints.push_back(objp);
        imgpoints.push_back(corners);
        used++;
    }

    if (used < 8) {
        std::cerr << "[ERR] not enough valid images. used=" << used
                  << ", total=" << files.size() << "\n";
        return 4;
    }

    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    const double rms = cv::calibrateCamera(
        objpoints, imgpoints, imageSize, K, dist, rvecs, tvecs);

    std::cout << "[INFO] total_images=" << files.size()
              << " used_images=" << used
              << " rms=" << rms << "\n";

    cv::FileStorage fs(outYaml, cv::FileStorage::WRITE);
    fs << "camera_matrix" << K;
    fs << "dist_coeffs" << dist;
    fs.release();

    std::cout << "[OK] saved: " << outYaml << "\n";

    if (!failed.empty()) {
        std::cout << "[INFO] failed_samples (first 20):\n";
        const size_t n = std::min<size_t>(20, failed.size());
        for (size_t i = 0; i < n; ++i) {
            std::cout << " - " << failed[i] << "\n";
        }
        if (failed.size() > 20) {
            std::cout << " ... and " << (failed.size() - 20) << " more\n";
        }
    }

    return 0;
}
