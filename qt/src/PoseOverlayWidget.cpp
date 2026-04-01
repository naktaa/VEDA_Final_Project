#include "PoseOverlayWidget.h"

#include <QPainter>
#include <QPen>

static const QVector<QPair<QString, QString>> kPoseEdges = {
    {"nose", "left_shoulder"},
    {"nose", "right_shoulder"},
    {"left_shoulder", "right_shoulder"},
    {"left_shoulder", "left_elbow"},
    {"left_elbow", "left_wrist"},
    {"right_shoulder", "right_elbow"},
    {"right_elbow", "right_wrist"},
    {"left_shoulder", "left_hip"},
    {"right_shoulder", "right_hip"},
    {"left_hip", "right_hip"},
    {"left_hip", "left_knee"},
    {"left_knee", "left_ankle"},
    {"right_hip", "right_knee"},
    {"right_knee", "right_ankle"}
};

PoseOverlayWidget::PoseOverlayWidget(QWidget *parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TranslucentBackground);
}

void PoseOverlayWidget::setPoseFrame(const PoseFrame &frame)
{
    m_frame = frame;
    update();
}

void PoseOverlayWidget::setVideoGeometry(const QRect &displayRect, const QSize &sourceFrameSize)
{
    m_videoRect = displayRect;
    if (sourceFrameSize.isValid()) m_sourceFrameSize = sourceFrameSize;
    update();
}

QPointF PoseOverlayWidget::mapPoint(const QPointF &src) const
{
    const QRect targetRect = m_videoRect.isValid() ? m_videoRect : rect();
    const QSize sourceSize = m_sourceFrameSize.isValid()
                                 ? m_sourceFrameSize
                                 : QSize(m_frame.frameW, m_frame.frameH);

    if (sourceSize.width() <= 0 || sourceSize.height() <= 0) {
        return QPointF(targetRect.left() + src.x(), targetRect.top() + src.y());
    }

    const double sx = static_cast<double>(targetRect.width()) / sourceSize.width();
    const double sy = static_cast<double>(targetRect.height()) / sourceSize.height();

    return QPointF(targetRect.left() + src.x() * sx,
                   targetRect.top() + src.y() * sy);
}

QRectF PoseOverlayWidget::mapRect(const QRect &src) const
{
    if (!src.isValid()) return {};
    return QRectF(mapPoint(QPointF(src.left(), src.top())),
                  mapPoint(QPointF(src.right(), src.bottom()))).normalized();
}

void PoseOverlayWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    if (!m_frame.person.detected)
        return;

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    drawBBox(p);
    drawSkeleton(p);
    drawLabel(p);
}

void PoseOverlayWidget::drawBBox(QPainter &p)
{
    if (!m_frame.person.bbox.isValid())
        return;

    p.setPen(QPen(QColor(255, 0, 0), 2));
    p.setBrush(Qt::NoBrush);
    p.drawRect(mapRect(m_frame.person.bbox));
}

void PoseOverlayWidget::drawSkeleton(QPainter &p)
{
    p.setPen(QPen(QColor(0, 255, 0), 2));

    for (const auto &edge : kPoseEdges) {
        if (!m_frame.person.keypoints.contains(edge.first)) continue;
        if (!m_frame.person.keypoints.contains(edge.second)) continue;

        const PoseKeypoint kp1 = m_frame.person.keypoints.value(edge.first);
        const PoseKeypoint kp2 = m_frame.person.keypoints.value(edge.second);

        if (!kp1.valid || !kp2.valid) continue;
        if (kp1.visibility < 0.5f || kp2.visibility < 0.5f) continue;

        p.drawLine(mapPoint(kp1.pt), mapPoint(kp2.pt));
    }

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 255, 0));

    for (auto it = m_frame.person.keypoints.begin(); it != m_frame.person.keypoints.end(); ++it) {
        const PoseKeypoint kp = it.value();
        if (!kp.valid || kp.visibility < 0.5f) continue;

        p.drawEllipse(mapPoint(kp.pt), 4, 4);
    }
}

void PoseOverlayWidget::drawLabel(QPainter &p)
{
    p.setPen(Qt::white);
    QFont f = font();
    f.setPointSize(11);
    f.setBold(true);
    p.setFont(f);

    const QString text = QString("%1 | Pose %2")
                             .arg(m_frame.cameraId.isEmpty() ? "pose" : m_frame.cameraId)
                             .arg(m_frame.person.score, 0, 'f', 2);
    p.drawText(12, 24, text);
}
