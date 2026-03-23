#include "GstVideoWidget.h"

#include <QDebug>
#include <QHash>
#include <QMutexLocker>
#include <QPainter>
#include <QTimer>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

static bool g_gst_inited = false;

namespace {

QHash<const GstVideoWidget*, bool> g_bgrOverrideByWidget;

bool shouldUseBgrOverride(const QString& url) {
    const QString normalized = url.trimmed().toLower();
    return normalized.contains(":8555/cam") ||
           normalized.contains(":8555/raw") ||
           normalized.endsWith("/cam") ||
           normalized.endsWith("/raw");
}

QImage copyRgbLikeFrame(const GstMapInfo& map,
                        int width,
                        int height,
                        int bytesPerLine,
                        bool forceBgrOverride) {
    if (forceBgrOverride) {
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
        QImage img((const uchar*)map.data, width, height, bytesPerLine, QImage::Format_BGR888);
        return img.copy();
#else
        QImage img((const uchar*)map.data, width, height, bytesPerLine, QImage::Format_RGB888);
        return img.rgbSwapped().copy();
#endif
    }

    QImage img((const uchar*)map.data, width, height, bytesPerLine, QImage::Format_RGB888);
    return img.copy();
}

} // namespace

GstVideoWidget::GstVideoWidget(QWidget* parent)
    : QWidget(parent) {
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setAutoFillBackground(true);
    setStyleSheet("background-color:black;");

    ensureGstInit();

    m_pullTimer = new QTimer(this);
    m_pullTimer->setTimerType(Qt::PreciseTimer);
    m_pullTimer->setInterval(15);
    connect(m_pullTimer, &QTimer::timeout, this, &GstVideoWidget::onPullFrame);
}

GstVideoWidget::~GstVideoWidget() {
    stopStream();
}

void GstVideoWidget::ensureGstInit() {
    if (!g_gst_inited) {
        gst_init(nullptr, nullptr);
        g_gst_inited = true;
    }
}

void GstVideoWidget::stopStream() {
    if (m_pullTimer) {
        m_pullTimer->stop();
    }
    g_bgrOverrideByWidget.remove(this);

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

void GstVideoWidget::startStream(const QString& url) {
    stopStream();

    const bool forceBgrOverride = shouldUseBgrOverride(url);
    g_bgrOverrideByWidget.insert(this, forceBgrOverride);

    const bool isRtsp = url.trimmed().startsWith("rtsp://", Qt::CaseInsensitive);

    QString pipeStr;
    if (isRtsp) {
        pipeStr = QString(
                      "rtspsrc location=\"%1\" protocols=tcp latency=60 "
                      "drop-on-latency=true udp-reconnect=true timeout=5000000 ! "
                      "rtph264depay ! "
                      "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                      "h264parse ! "
                      "avdec_h264 ! "
                      "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                      "videoconvert ! "
                      "video/x-raw,format=RGB ! "
                      "videocrop name=zcrop top=0 bottom=0 left=0 right=0 ! "
                      "appsink name=vsink sync=false max-buffers=1 drop=true "
                      "emit-signals=false enable-last-sample=false wait-on-eos=false")
                      .arg(url);

        // UDP test pipeline:
        // pipeStr = QString(
        //               "rtspsrc location=\"%1\" protocols=udp latency=20 "
        //               "drop-on-latency=true udp-reconnect=true timeout=5000000 ! "
        //               "rtph264depay ! "
        //               "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
        //               "h264parse ! "
        //               "avdec_h264 ! "
        //               "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
        //               "videoconvert ! "
        //               "video/x-raw,format=RGB ! "
        //               "videocrop name=zcrop top=0 bottom=0 left=0 right=0 ! "
        //               "appsink name=vsink sync=false max-buffers=1 drop=true "
        //               "emit-signals=false enable-last-sample=false wait-on-eos=false")
        //               .arg(url);
    } else {
        QString source = QString("uridecodebin uri=\"%1\"").arg(url);
        pipeStr = QString(
                      "%1 ! "
                      "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                      "videoconvert ! "
                      "videocrop name=zcrop top=0 bottom=0 left=0 right=0 ! "
                      "video/x-raw,format=RGB ! "
                      "appsink name=vsink sync=false max-buffers=1 drop=true "
                      "emit-signals=false enable-last-sample=false wait-on-eos=false")
                      .arg(source);
    }

    qDebug() << "[GstVideoWidget] startStream:" << url;
    qDebug() << "[GstVideoWidget] color mode:" << (forceBgrOverride ? "bgr-override" : "normal-rgb");
    qDebug() << "[GstVideoWidget] pipeline:" << pipeStr;

    GError* err = nullptr;
    m_pipeline = gst_parse_launch(pipeStr.toUtf8().constData(), &err);
    if (!m_pipeline) {
        qDebug() << "[GstVideoWidget] pipeline create failed:" << (err ? err->message : "unknown");
        if (err) {
            g_error_free(err);
        }
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

    GstAppSink* as = GST_APP_SINK(m_appsink);
    gst_app_sink_set_drop(as, TRUE);
    gst_app_sink_set_max_buffers(as, 1);
    gst_app_sink_set_emit_signals(as, FALSE);
    g_object_set(G_OBJECT(as),
                 "enable-last-sample", FALSE,
                 "wait-on-eos", FALSE,
                 "sync", FALSE,
                 nullptr);

    gst_element_set_state(m_pipeline, GST_STATE_PLAYING);

    resetZoom();
    m_pullTimer->start();
}

void GstVideoWidget::onPullFrame() {
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
                if (err) {
                    g_error_free(err);
                }
                if (dbg) {
                    g_free(dbg);
                }
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError* err = nullptr;
                gchar* dbg = nullptr;
                gst_message_parse_warning(msg, &err, &dbg);
                qDebug() << "[GstVideoWidget] WARNING:"
                         << (err ? err->message : "unknown")
                         << (dbg ? dbg : "");
                if (err) {
                    g_error_free(err);
                }
                if (dbg) {
                    g_free(dbg);
                }
                break;
            }
            default:
                break;
            }
            gst_message_unref(msg);
        }
    }

    if (!m_appsink) {
        return;
    }

    const bool forceBgrOverride = g_bgrOverrideByWidget.value(this, false);

    GstSample* sample = nullptr;
    for (;;) {
        GstSample* next = gst_app_sink_try_pull_sample(GST_APP_SINK(m_appsink), 0);
        if (!next) {
            break;
        }
        if (sample) {
            gst_sample_unref(sample);
        }
        sample = next;
    }
    if (!sample) {
        return;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    if (!buffer || !caps) {
        gst_sample_unref(sample);
        return;
    }

    GstStructure* s = gst_caps_get_structure(caps, 0);
    int w = 0;
    int h = 0;
    gst_structure_get_int(s, "width", &w);
    gst_structure_get_int(s, "height", &h);

    if (m_videoW == 0 && m_videoH == 0) {
        gchar* capsStr = gst_caps_to_string(caps);
        qDebug() << "[GstVideoWidget] sample caps =" << (capsStr ? capsStr : "null");
        if (capsStr) {
            g_free(capsStr);
        }
    }

    if (w <= 0 || h <= 0) {
        gst_sample_unref(sample);
        return;
    }

    m_videoW = w;
    m_videoH = h;

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return;
    }

    GstVideoInfo info;
    const bool infoOk = gst_video_info_from_caps(&info, caps);
    const int bytesPerLine =
        (infoOk && info.stride[0] > 0) ? static_cast<int>(info.stride[0]) : (w * 3);

    QImage copy = copyRgbLikeFrame(map, w, h, bytesPerLine, forceBgrOverride);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    {
        QMutexLocker lk(&m_frameMtx);
        m_frame = copy;
    }
    update();
}

void GstVideoWidget::paintEvent(QPaintEvent* e) {
    Q_UNUSED(e);

    QPainter p(this);
    p.fillRect(rect(), Qt::black);

    QImage frameCopy;
    {
        QMutexLocker lk(&m_frameMtx);
        frameCopy = m_frame;
    }
    if (frameCopy.isNull()) {
        return;
    }

    QSize target = frameCopy.size();
    target.scale(size(), Qt::KeepAspectRatio);

    QRect dst(QPoint(0, 0), target);
    dst.moveCenter(rect().center());

    p.drawImage(dst, frameCopy);
}

bool GstVideoWidget::widgetToImagePoint(const QPointF& widgetPos, QPointF& imagePos) const {
    QImage frameCopy;
    {
        QMutexLocker lk(&m_frameMtx);
        frameCopy = m_frame;
    }
    if (frameCopy.isNull()) {
        return false;
    }

    QSize target = frameCopy.size();
    target.scale(size(), Qt::KeepAspectRatio);
    QRectF drawRect(QPointF(0, 0), target);
    drawRect.moveCenter(rect().center());

    if (!drawRect.contains(widgetPos)) {
        return false;
    }

    const double nx = (widgetPos.x() - drawRect.left()) / drawRect.width();
    const double ny = (widgetPos.y() - drawRect.top()) / drawRect.height();

    imagePos = QPointF(nx * frameCopy.width(), ny * frameCopy.height());
    return true;
}

QRect GstVideoWidget::videoDisplayRect() const {
    QImage frameCopy;
    {
        QMutexLocker lk(&m_frameMtx);
        frameCopy = m_frame;
    }
    if (frameCopy.isNull()) {
        return rect();
    }

    QSize target = frameCopy.size();
    target.scale(size(), Qt::KeepAspectRatio);

    QRect dst(QPoint(0, 0), target);
    dst.moveCenter(rect().center());
    return dst;
}

QSize GstVideoWidget::currentFrameSize() const {
    QMutexLocker lk(&m_frameMtx);
    return m_frame.size();
}

void GstVideoWidget::setDigitalZoom(double zoom, double cx, double cy) {
    if (zoom < 1.0) {
        zoom = 1.0;
    }
    if (zoom > 6.0) {
        zoom = 6.0;
    }
    if (cx < 0.0) {
        cx = 0.0;
    }
    if (cx > 1.0) {
        cx = 1.0;
    }
    if (cy < 0.0) {
        cy = 0.0;
    }
    if (cy > 1.0) {
        cy = 1.0;
    }

    m_zoom = zoom;
    m_cx = cx;
    m_cy = cy;

    updateCrop(m_zoom, m_cx, m_cy);
}

void GstVideoWidget::resetZoom() {
    setDigitalZoom(1.0, 0.5, 0.5);
}

void GstVideoWidget::updateVideoSizeFromCaps() {
}

void GstVideoWidget::updateCrop(double zoom, double cx, double cy) {
    if (!m_crop) {
        return;
    }

    int W = 640;
    int H = 480;
    {
        QMutexLocker lk(&m_frameMtx);
        if (!m_frame.isNull()) {
            W = m_frame.width();
            H = m_frame.height();
        }
    }

    if (zoom <= 1.0) {
        g_object_set(G_OBJECT(m_crop),
                     "left", 0,
                     "right", 0,
                     "top", 0,
                     "bottom", 0,
                     nullptr);
        return;
    }

    int outW = static_cast<int>(W / zoom);
    int outH = static_cast<int>(H / zoom);

    if (outW < 16) {
        outW = 16;
    }
    if (outH < 16) {
        outH = 16;
    }

    const int cropW = W - outW;
    const int cropH = H - outH;

    int desiredLeft = static_cast<int>(W * cx - outW / 2);
    int desiredTop = static_cast<int>(H * cy - outH / 2);

    if (desiredLeft < 0) {
        desiredLeft = 0;
    }
    if (desiredLeft > cropW) {
        desiredLeft = cropW;
    }

    if (desiredTop < 0) {
        desiredTop = 0;
    }
    if (desiredTop > cropH) {
        desiredTop = cropH;
    }

    const int left = desiredLeft;
    const int right = cropW - left;
    const int top = desiredTop;
    const int bottom = cropH - top;

    g_object_set(G_OBJECT(m_crop),
                 "left", left,
                 "right", right,
                 "top", top,
                 "bottom", bottom,
                 nullptr);
}
