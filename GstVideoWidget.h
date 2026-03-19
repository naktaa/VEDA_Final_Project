#ifndef GSTVIDEOWIDGET_H
#define GSTVIDEOWIDGET_H
#pragma once

#include <QWidget>
#include <QImage>
#include <QMutex>
#include <QPointF>
#include <QTimer>
#include <QString>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class GstVideoWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GstVideoWidget(QWidget *parent = nullptr);
    ~GstVideoWidget();

    void startStream(const QString &url);
    void stopStream();

    // 디지털 줌
    void setDigitalZoom(double zoom, double cx = 0.5, double cy = 0.5);
    void resetZoom();
    bool widgetToImagePoint(const QPointF& widgetPos, QPointF& imagePos) const;
    QRect videoDisplayRect() const;
    QSize currentFrameSize() const;

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    static void ensureGstInit();

    void onPullFrame();
    void updateCrop(double zoom, double cx, double cy);
    void updateVideoSizeFromCaps();  // 유지용 (필요시 확장)

private:
    // GStreamer
    GstElement *m_pipeline = nullptr;
    GstElement *m_appsink = nullptr;
    GstElement *m_crop = nullptr;
    GstBus *m_bus = nullptr;

    // 프레임 저장
    QImage m_frame;
    mutable QMutex m_frameMtx;
    QTimer *m_pullTimer = nullptr;

    // video size
    int m_videoW = 0;
    int m_videoH = 0;

    // zoom state
    double m_zoom = 1.0;
    double m_cx = 0.5;
    double m_cy = 0.5;
};

#endif // GSTVIDEOWIDGET_H
