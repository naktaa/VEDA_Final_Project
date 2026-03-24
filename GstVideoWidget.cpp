#include "GstVideoWidget.h"

#include <QDebug>
#include <QPainter>
#include <QTimer>
#include <QMutexLocker>
#include <QUrl>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

static bool g_gst_inited = false;

namespace {

QString buildStandardPipeline(const QString& url, int outWidth)
{
    const bool isRtsp = url.trimmed().startsWith("rtsp://", Qt::CaseInsensitive);
    QString source = QString("uridecodebin uri=\"%1\"").arg(url);
    if (isRtsp) {
        source += " source::protocols=tcp source::latency=50";
    }

    return QString(
               "%1 ! "
               "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
               "videoconvert ! "
               "videocrop name=zcrop top=0 bottom=0 left=0 right=0 ! "
               "videoscale ! "
               "video/x-raw,format=BGRA,width=%2,pixel-aspect-ratio=1/1 ! "
               "appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=false")
        .arg(source)
        .arg(outWidth);
}

QString buildRcTankPipeline(const QString& url, int outWidth)
{
    const QString trimmed = url.trimmed();
    if (trimmed.startsWith("udp://5000", Qt::CaseInsensitive)) {
        const QUrl parsed(trimmed);
        const int port = parsed.port(5000);
        return QString(
                   "udpsrc port=%1 caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000\" ! "
                   "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                   "rtpjitterbuffer la  tency=10 drop-on-latency=true ! "
                   "rtph264depay ! "
                   "h264parse ! "
                   "avdec_h264 ! "
                   "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                   "videoconvert ! "
                   "videocrop name=zcrop top=0 bottom=0 left=0 right=0 ! "
                   "videoscale ! "
                   "video/x-raw,format=BGRA,width=%2,pixel-aspect-ratio=1/1 ! "
                   "appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=false")
            .arg(port)
            .arg(outWidth);
    }

    if (trimmed.startsWith("rtsp://", Qt::CaseInsensitive)) {
        return QString(
                   "uridecodebin uri=\"%1\" source::protocols=udp source::latency=20 ! "
                   "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                   "videoconvert ! "
                   "videocrop name=zcrop top=0 bottom=0 left=0 right=0 ! "
                   "videoscale ! "
                   "video/x-raw,format=BGRA,width=%2,pixel-aspect-ratio=1/1 ! "
                   "appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=false")
            .arg(trimmed)
            .arg(outWidth);
    }

    return buildStandardPipeline(trimmed, outWidth);
}

}

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
    const int OUT_W = m_preferredOutputWidth;

    // ??appsink�??�레?�을 Qt�?가?�온??(?�버?�이 100% 가??
    const QString pipeStr =
        (m_streamProfile == StreamProfile::RcTankLowLatency)
            ? buildRcTankPipeline(url, OUT_W)
            : buildStandardPipeline(url, OUT_W);

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

void GstVideoWidget::setStreamProfile(StreamProfile profile)
{
    m_streamProfile = profile;
}

GstVideoWidget::StreamProfile GstVideoWidget::streamProfile() const
{
    return m_streamProfile;
}

void GstVideoWidget::setGrayscaleEnabled(bool enabled)
{
    setDisplayMode(enabled ? DisplayMode::Grayscale : DisplayMode::Normal);
}

void GstVideoWidget::setDisplayMode(DisplayMode mode)
{
    if (m_displayMode == mode) return;
    m_displayMode = mode;
    update();
}

bool GstVideoWidget::isGrayscaleEnabled() const
{
    return m_displayMode == DisplayMode::Grayscale;
}

GstVideoWidget::DisplayMode GstVideoWidget::displayMode() const
{
    return m_displayMode;
}

void GstVideoWidget::setPreferredOutputWidth(int width)
{
    if (width < 160) width = 160;
    m_preferredOutputWidth = width;
}

int GstVideoWidget::preferredOutputWidth() const
{
    return m_preferredOutputWidth;
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
    QImage copy = img.copy(); // ???�정???�선 (?�능보다 ?�실  )

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    {
        QMutexLocker lk(&m_frameMtx);
        m_prevFrame = m_frame;
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

    if (m_displayMode == DisplayMode::Grayscale) {
        frameCopy = frameCopy.convertToFormat(QImage::Format_Grayscale8);
    } else if (m_displayMode == DisplayMode::HighContrast) {
        frameCopy = frameCopy.convertToFormat(QImage::Format_Grayscale8);
        frameCopy = frameCopy.convertToFormat(QImage::Format_ARGB32);

        const int width = frameCopy.width();
        const int height = frameCopy.height();
        for (int y = 0; y < height; ++y) {
            QRgb* line = reinterpret_cast<QRgb*>(frameCopy.scanLine(y));
            for (int x = 0; x < width; ++x) {
                const int v = qRed(line[x]);
                const int boosted = (v < 72) ? 0 : ((v > 168) ? 255 : qBound(0, (v - 72) * 3, 255));
                line[x] = qRgba(boosted, boosted, boosted, qAlpha(line[x]));
            }
        }
    } else if (m_displayMode == DisplayMode::EdgeEnhanced) {
        const QImage gray = frameCopy.convertToFormat(QImage::Format_Grayscale8);
        QImage edge(gray.size(), QImage::Format_ARGB32);
        edge.fill(Qt::black);

        const int width = gray.width();
        const int height = gray.height();
        for (int y = 1; y < height - 1; ++y) {
            const uchar* prev = gray.constScanLine(y - 1);
            const uchar* curr = gray.constScanLine(y);
            const uchar* next = gray.constScanLine(y + 1);
            QRgb* out = reinterpret_cast<QRgb*>(edge.scanLine(y));

            for (int x = 1; x < width - 1; ++x) {
                const int gx = -prev[x - 1] + prev[x + 1]
                             - 2 * curr[x - 1] + 2 * curr[x + 1]
                             - next[x - 1] + next[x + 1];
                const int gy = -prev[x - 1] - 2 * prev[x] - prev[x + 1]
                             + next[x - 1] + 2 * next[x] + next[x + 1];
                const int mag = qBound(0, qAbs(gx) + qAbs(gy), 255);
                const int edgeValue = (mag < 48) ? 0 : qBound(0, (mag - 48) * 2, 255);
                out[x] = qRgba(edgeValue, edgeValue, edgeValue, 255);
            }
        }

        frameCopy = edge;
    } else if (m_displayMode == DisplayMode::Inverted) {
        frameCopy.invertPixels();
    } else if (m_displayMode == DisplayMode::ZoomCrop) {
        const int cropW = qMax(32, frameCopy.width() / 2);
        const int cropH = qMax(32, frameCopy.height() / 2);
        frameCopy = frameCopy.copy((frameCopy.width() - cropW) / 2,
                                   (frameCopy.height() - cropH) / 2,
                                   cropW,
                                   cropH);
    } else if (m_displayMode == DisplayMode::MotionHighlight) {
        QImage prevCopy;
        {
            QMutexLocker lk(&m_frameMtx);
            prevCopy = m_prevFrame;
        }

        if (!prevCopy.isNull() && prevCopy.size() == frameCopy.size()) {
            const QImage currGray = frameCopy.convertToFormat(QImage::Format_Grayscale8);
            const QImage prevGray = prevCopy.convertToFormat(QImage::Format_Grayscale8);
            QImage motion(frameCopy.size(), QImage::Format_ARGB32);
            motion.fill(Qt::black);

            for (int y = 0; y < motion.height(); ++y) {
                const uchar* curr = currGray.constScanLine(y);
                const uchar* prev = prevGray.constScanLine(y);
                QRgb* out = reinterpret_cast<QRgb*>(motion.scanLine(y));
                for (int x = 0; x < motion.width(); ++x) {
                    const int diff = qAbs(int(curr[x]) - int(prev[x]));
                    const int value = diff < 20 ? 0 : qBound(0, (diff - 20) * 6, 255);
                    out[x] = qRgba(value, value, value, 255);
                }
            }
            frameCopy = motion;
        } else {
            frameCopy = frameCopy.convertToFormat(QImage::Format_Grayscale8);
        }
    }

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
