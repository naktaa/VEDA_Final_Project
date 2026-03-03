#ifndef HOMOGRAPHYCALIB_H
#define HOMOGRAPHYCALIB_H

#include <QPointF>
#include <QTransform>
#include <QVector>

class HomographyCalib
{
public:
    // worldPts: meter 좌표
    // imgPts:   CCTV 프레임 픽셀 좌표
    // outH:     world -> image
    static bool computeWorldToImage(
        const QVector<QPointF>& worldPts,
        const QVector<QPointF>& imgPts,
        QTransform& outH,
        bool useRansac = true,
        double ransacReprojThresholdPx = 3.0
        );

    // ✅ 추가: image -> world 변환행렬(역행렬)
    static bool computeImageToWorld(
        const QVector<QPointF>& worldPts,
        const QVector<QPointF>& imgPts,
        QTransform& outH_inv,
        bool useRansac = true,
        double ransacReprojThresholdPx = 3.0
        );
};

#endif
