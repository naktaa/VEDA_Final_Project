#include "HomographyCalib.h"

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

static inline std::vector<cv::Point2f> toCvPts(const QVector<QPointF>& pts)
{
    std::vector<cv::Point2f> v;
    v.reserve(pts.size());
    for (const auto& p : pts)
        v.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
    return v;
}

static inline QTransform cvHToQTransform(const cv::Mat& H)
{
    const double h11 = H.at<double>(0,0);
    const double h12 = H.at<double>(0,1);
    const double h13 = H.at<double>(0,2);
    const double h21 = H.at<double>(1,0);
    const double h22 = H.at<double>(1,1);
    const double h23 = H.at<double>(1,2);
    const double h31 = H.at<double>(2,0);
    const double h32 = H.at<double>(2,1);
    const double h33 = H.at<double>(2,2);

    return QTransform(h11, h21, h31,
                      h12, h22, h32,
                      h13, h23, h33);
}

bool HomographyCalib::computeWorldToImage(
    const QVector<QPointF>& worldPts,
    const QVector<QPointF>& imgPts,
    QTransform& outH,
    bool useRansac,
    double ransacReprojThresholdPx
    ){
    if (worldPts.size() < 4 || imgPts.size() < 4) return false;
    if (worldPts.size() != imgPts.size()) return false;

    auto w = toCvPts(worldPts);
    auto i = toCvPts(imgPts);

    cv::Mat mask;
    cv::Mat H;

    if (useRansac) {
        H = cv::findHomography(w, i, cv::RANSAC, ransacReprojThresholdPx, mask);
    } else {
        H = cv::findHomography(w, i, 0);
    }

    if (H.empty() || H.rows != 3 || H.cols != 3) return false;

    H.convertTo(H, CV_64F);
    outH = cvHToQTransform(H);

    return true; // ✅ 이게 맞음
}

// ✅ 추가: image -> world (역행렬)
bool HomographyCalib::computeImageToWorld(
    const QVector<QPointF>& worldPts,
    const QVector<QPointF>& imgPts,
    QTransform& outH_inv,
    bool useRansac,
    double ransacReprojThresholdPx
    ){
    QTransform H_w2i;
    if (!computeWorldToImage(worldPts, imgPts, H_w2i, useRansac, ransacReprojThresholdPx))
        return false;

    bool ok = false;
    QTransform H_i2w = H_w2i.inverted(&ok);
    if (!ok) return false;

    outH_inv = H_i2w;
    return true;
}
