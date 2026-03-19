#include "GstVideoWidget.h"

#include <QDebug>
#include <QPainter>
#include <QTimer>
#include <QMutexLocker>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

static bool g_gst_inited = false;

GstVideoWidget::GstVideoWidget(QWidget *parent)
    : QWidget(parent)
{
    // ???�제 native overlay ?�요 ?�음 (?�버?�이 ?�해 ?�거)
    // setAttribute(Qt::WA_NativeWindow);
    // setAttribute(Qt::WA_PaintOnScreen);

    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setAutoFillBackground(true);
    setStyleSheet("background-color:black;");

    ensureGstInit();

    m_pullTimer = new QTimer(this);
    // ??? ?�상??기�? 30fps ?�도�?충분 (추측)
    m_pullTimer->setInterval(33);
    connect(m_pullTimer, &QTimer::timeout, this, &GstVideoWidget::onPullFrame);
}

GstVideoWidget::~GstVideoWidget()
{
    stopStream();
}

void GstVideoWidget::ensureGstInit()
{
    if (!g_gst_inited) {
        gst_init(nullptr, nullptr);
        g_gst_inited = true;
    }
}

void GstVideoWidget::stopStream()
{
    if (m_pullTimer) m_pullTimer->stop();

    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
    }

    if (m_bus) {
        gst_object_unref(m_bus);
        m_bus = nullptr;
    }
    if (m_crop) {
        gst_object_unref(m_crop);
        m_crop = nullptr;
    }
    if (m_appsink) {
        gst_object_unref(m_appsink);
        m_appsink = nullptr;
    }
    if (m_pipeline) {
        gst_object_unref(m_pipeline);
        m_pipeline = nullptr;
    }

    {
        QMutexLocker lk(&m_frameMtx);
        m_frame = QImage();
    }
    update();

    m_videoW = m_videoH = 0;
    m_zoom = 1.0;
    m_cx = m_cy = 0.5;
}

void GstVideoWidget::startStream(const QString &url)
{
    stopStream();

    // ???�상????�� 고정 (기본 640x360)
    // ?�요?�면 854x480 or 1280x720�?변�?가??
    const int OUT_W = 1280;
    const int OUT_H = 960;

    // ??appsink�??�레?�을 Qt�?가?�온??(?�버?�이 100% 가??
    const bool isRtsp = url.trimmed().startsWith("rtsp://", Qt::CaseInsensitive);
    QString source = QString("uridecodebin uri=\"%1\"").arg(url);
    if (isRtsp) {
        // Prefer RTSP over TCP for embedded cameras/relays that stall on UDP.
        source += " source::protocols=tcp source::latency=100";
    }

    QString pipeStr = QString(
                          "%1 ! "
                          "queue max-size-buffers=2 ! "
                          "videoconvert ! "
                          "videocrop name=zcrop top=0 bottom=0 left=0 right=0 ! "
                          "videoscale ! "
                          "video/x-raw,format=BGRA,width=%2,height=%3 ! "
                          "appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=false"
                          ).arg(source).arg(OUT_W).arg(OUT_H);

    qDebug() << "[GstVideoWidget] startStream:" << url;
    qDebug() << "[GstVideoWidget] pipeline:" << pipeStr;

    GError *err = nullptr;
    m_pipeline = gst_parse_launch(pipeStr.toUtf8().constData(), &err);
    if (!m_pipeline) {
        qDebug() << "[GstVideoWidget] pipeline create failed:" << (err ? err->message : "unknown");
        if (err) g_error_free(err);
        return;
    }

    m_crop = gst_bin_get_by_name(GST_BIN(m_pipeline), "zcrop");
    m_appsink = gst_bin_get_by_name(GST_BIN(m_pipeline), "vsink");
    m_bus = gst_element_get_bus(m_pipeline);

    if (!m_appsink) {
        qDebug() << "[GstVideoWidget] appsink not found";
        stopStream();
        return;
    }

    // appsink ?�정 (?�전?�게 ??�???
    GstAppSink* as = GST_APP_SINK(m_appsink);
    gst_app_sink_set_drop(as, TRUE);
    gst_app_sink_set_max_buffers(as, 1);
    gst_app_sink_set_emit_signals(as, FALSE);

    gst_element_set_state(m_pipeline, GST_STATE_PLAYING);

    // ??�??�작?� �??�제 ?�태
    resetZoom();

    // ??pull timer ?�작
    m_pullTimer->start();
}

void GstVideoWidget::onPullFrame()
{
    if (m_bus) {
        while (GstMessage* msg = gst_bus_pop(m_bus)) {
            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError* err = nullptr;
                gchar* dbg = nullptr;
                gst_message_parse_error(msg, &err, &dbg);
                qDebug() << "[GstVideoWidget] ERROR:"
                         << (err ? err->message : "unknown")
                         << (dbg ? dbg : "");
                if (err) g_error_free(err);
                if (dbg) g_free(dbg);
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError* err = nullptr;
                gchar* dbg = nullptr;
                gst_message_parse_warning(msg, &err, &dbg);
                qDebug() << "[GstVideoWidget] WARNING:"
                         << (err ? err->message : "unknown")
                         << (dbg ? dbg : "");
                if (err) g_error_free(err);
                if (dbg) g_free(dbg);
                break;
            }
            default:
                break;
            }
            gst_message_unref(msg);
        }
    }

    if (!m_appsink) return;

    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(m_appsink), 0);
    if (!sample) return;

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    if (!buffer || !caps) {
        gst_sample_unref(sample);
        return;
    }

    GstStructure* s = gst_caps_get_structure(caps, 0);
    int w = 0, h = 0;
    gst_structure_get_int(s, "width", &w);
    gst_structure_get_int(s, "height", &h);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return;
    }

    // BGRA ?�맷?�로 받았?�니 QImage::Format_ARGB32 ?�용 가??
    // ?�️ map.data??gst 버퍼 메모리라??sample unref ??무효가 ?????�음 ??deep copy
    QImage img((const uchar*)map.data, w, h, QImage::Format_ARGB32);
    QImage copy = img.copy(); // ???�정???�선 (?�능보다 ?�실)

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    {
        QMutexLocker lk(&m_frameMtx);
        m_frame = copy;
    }
    update();
}

void GstVideoWidget::paintEvent(QPaintEvent *e)
{
    Q_UNUSED(e);

    QPainter p(this);
    p.fillRect(rect(), Qt::black);

    QImage frameCopy;
    {
        QMutexLocker lk(&m_frameMtx);
        frameCopy = m_frame;
    }
    if (frameCopy.isNull()) return;

    // ???�젯 ?�기??맞게 letterbox�??�시
    QSize target = frameCopy.size();
    target.scale(size(), Qt::KeepAspectRatio);

    QRect dst(QPoint(0,0), target);
    dst.moveCenter(rect().center());

    p.drawImage(dst, frameCopy);
}

bool GstVideoWidget::widgetToImagePoint(const QPointF& widgetPos, QPointF& imagePos) const
{
    QImage frameCopy;
    {
        QMutexLocker lk(&m_frameMtx);
        frameCopy = m_frame;
    }
    if (frameCopy.isNull()) return false;

    QSize target = frameCopy.size();
    target.scale(size(), Qt::KeepAspectRatio);
    QRectF drawRect(QPointF(0, 0), target);
    drawRect.moveCenter(rect().center());

    if (!drawRect.contains(widgetPos)) return false;

    const double nx = (widgetPos.x() - drawRect.left()) / drawRect.width();
    const double ny = (widgetPos.y() - drawRect.top()) / drawRect.height();

    imagePos = QPointF(nx * frameCopy.width(), ny * frameCopy.height());
    return true;
}

QRect GstVideoWidget::videoDisplayRect() const
{
    QImage frameCopy;
    {
        QMutexLocker lk(&m_frameMtx);
        frameCopy = m_frame;
    }
    if (frameCopy.isNull()) return rect();

    QSize target = frameCopy.size();
    target.scale(size(), Qt::KeepAspectRatio);

    QRect dst(QPoint(0,0), target);
    dst.moveCenter(rect().center());
    return dst;
}

QSize GstVideoWidget::currentFrameSize() const
{
    QMutexLocker lk(&m_frameMtx);
    return m_frame.size();
}

// ---- zoom (기존 로직 ?��?) ----
void GstVideoWidget::setDigitalZoom(double zoom, double cx, double cy)
{
    if (zoom < 1.0) zoom = 1.0;
    if (zoom > 6.0) zoom = 6.0;
    if (cx < 0.0) cx = 0.0; if (cx > 1.0) cx = 1.0;
    if (cy < 0.0) cy = 0.0; if (cy > 1.0) cy = 1.0;

    m_zoom = zoom;
    m_cx = cx;
    m_cy = cy;

    updateCrop(m_zoom, m_cx, m_cy);
}

void GstVideoWidget::resetZoom()
{
    setDigitalZoom(1.0, 0.5, 0.5);
}

void GstVideoWidget::updateVideoSizeFromCaps()
{
    // appsink�?받으므�?굳이 ?�요 ?��?�? 기존 ?�터?�이???��???
}

void GstVideoWidget::updateCrop(double zoom, double cx, double cy)
{
    if (!m_crop) return;

    // appsink 캡스 기�? OUT_W/OUT_H�??�어?��?�?crop 기�???�??�상??
    // ?�기?�는 ?�재 ?�레???�기�??�용 (?�으�?OUT_W/OUT_H 가??
    int W = 782, H = 592;
    {
        QMutexLocker lk(&m_frameMtx);
        if (!m_frame.isNull()) { W = m_frame.width(); H = m_frame.height(); }
    }

    if (zoom <= 1.0) {
        g_object_set(G_OBJECT(m_crop),
                     "left", 0, "right", 0, "top", 0, "bottom", 0, nullptr);
        return;
    }

    int outW = (int)(W / zoom);
    int outH = (int)(H / zoom);

    if (outW < 16) outW = 16;
    if (outH < 16) outH = 16;

    int cropW = W - outW;
    int cropH = H - outH;

    int desiredLeft = (int)(W * cx - outW / 2);
    int desiredTop  = (int)(H * cy - outH / 2);

    if (desiredLeft < 0) desiredLeft = 0;
    if (desiredLeft > cropW) desiredLeft = cropW;

    if (desiredTop < 0) desiredTop = 0;
    if (desiredTop > cropH) desiredTop = cropH;

    int left   = desiredLeft;
    int right  = cropW - left;
    int top    = desiredTop;
    int bottom = cropH - top;

    g_object_set(G_OBJECT(m_crop),
                 "left", left,
                 "right", right,
                 "top", top,
                 "bottom", bottom,
                 nullptr);
}
