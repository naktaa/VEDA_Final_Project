#include "HumanBoxOverlayWidget.h"

#include <QPainter>
#include <QPen>

HumanBoxOverlayWidget::HumanBoxOverlayWidget(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TranslucentBackground);
}

void HumanBoxOverlayWidget::setHumanBox(const QRect& bbox, bool visible)
{
    if (visible && bbox.isValid()) {
        m_boxes = {bbox};
    } else {
        m_boxes.clear();
    }
    m_visible = visible && !m_boxes.isEmpty();
    update();
}

void HumanBoxOverlayWidget::setHumanBoxes(const QVector<QRect>& boxes, bool visible)
{
    QVector<QRect> nextBoxes;
    if (visible) {
        nextBoxes.reserve(boxes.size());
        for (const QRect& box : boxes) {
            if (box.isValid()) nextBoxes.push_back(box);
        }
    }
    const bool nextVisible = visible && !nextBoxes.isEmpty();
    if (m_boxes == nextBoxes && m_visible == nextVisible) return;
    m_boxes = std::move(nextBoxes);
    m_visible = nextVisible;
    update();
}

void HumanBoxOverlayWidget::setVideoGeometry(const QRect& displayRect, const QSize& sourceFrameSize)
{
    m_videoRect = displayRect;
    if (sourceFrameSize.isValid()) m_sourceFrameSize = sourceFrameSize;
    update();
}

void HumanBoxOverlayWidget::setBoxSourceSize(const QSize& sourceFrameSize)
{
    if (!sourceFrameSize.isValid() || sourceFrameSize == m_sourceFrameSize) return;
    m_sourceFrameSize = sourceFrameSize;
    update();
}

QRectF HumanBoxOverlayWidget::mapRect(const QRect& src) const
{
    if (!src.isValid()) return {};

    const QRect targetRect = m_videoRect.isValid() ? m_videoRect : rect();
    if (!m_sourceFrameSize.isValid() || m_sourceFrameSize.width() <= 0 || m_sourceFrameSize.height() <= 0) {
        return QRectF(src);
    }

    const double sx = static_cast<double>(targetRect.width()) / m_sourceFrameSize.width();
    const double sy = static_cast<double>(targetRect.height()) / m_sourceFrameSize.height();

    return QRectF(
        targetRect.left() + src.left() * sx,
        targetRect.top() + src.top() * sy,
        src.width() * sx,
        src.height() * sy
    ).normalized();
}

void HumanBoxOverlayWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    if (!m_visible || m_boxes.isEmpty()) return;

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setPen(QPen(QColor(120, 255, 120), 3));
    p.setBrush(Qt::NoBrush);
    for (const QRect& box : m_boxes) {
        if (!box.isValid()) continue;
        p.drawRect(mapRect(box));
    }
}
