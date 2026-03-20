#include "HumanBoxOverlayWidget.h"

#include <algorithm>
#include <QPainter>
#include <QPen>

namespace {
constexpr double kHumanBoxOutwardScale = 1.03;
}

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
    const QRect nextVideoRect = displayRect;
    const QSize nextSourceFrameSize = sourceFrameSize.isValid() ? sourceFrameSize : m_sourceFrameSize;
    if (m_videoRect == nextVideoRect && m_sourceFrameSize == nextSourceFrameSize) return;

    m_videoRect = nextVideoRect;
    m_sourceFrameSize = nextSourceFrameSize;
    update();
}

void HumanBoxOverlayWidget::setBoxSourceSize(const QSize& sourceFrameSize)
{
    if (!sourceFrameSize.isValid() || sourceFrameSize == m_sourceFrameSize) return;
    m_sourceFrameSize = sourceFrameSize;
    update();
}

void HumanBoxOverlayWidget::setBoxSourceOffset(const QPointF& sourceOffset)
{
    if (m_sourceOffset == sourceOffset) return;
    m_sourceOffset = sourceOffset;
    update();
}

void HumanBoxOverlayWidget::setBoxPerspectiveOffset(const QPointF& perspectiveOffset)
{
    if (m_perspectiveOffset == perspectiveOffset) return;
    m_perspectiveOffset = perspectiveOffset;
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
    const double centerNormX = std::clamp(
        static_cast<double>(src.center().x()) / m_sourceFrameSize.width(), 0.0, 1.0);
    const double centerNormY = std::clamp(
        static_cast<double>(src.center().y()) / m_sourceFrameSize.height(), 0.0, 1.0);
    const double rightBias = std::clamp((centerNormX - 0.5) * 2.0, 0.0, 1.0);
    const double localOffsetX =
        m_sourceOffset.x() +
        (m_perspectiveOffset.x() * centerNormX) +
        (m_perspectiveOffset.x() * 3.0 * rightBias);
    const double localOffsetY = m_sourceOffset.y() + (m_perspectiveOffset.y() * centerNormY);

    const double srcCenterX = m_sourceFrameSize.width() * 0.5;
    const double srcCenterY = m_sourceFrameSize.height() * 0.5;

    const double adjustedLeft =
        srcCenterX + (((src.left() + localOffsetX) - srcCenterX) * kHumanBoxOutwardScale);
    const double adjustedTop =
        srcCenterY + (((src.top() + localOffsetY) - srcCenterY) * kHumanBoxOutwardScale);
    const double adjustedWidth = src.width() * kHumanBoxOutwardScale;
    const double adjustedHeight = src.height() * kHumanBoxOutwardScale;

    return QRectF(
        targetRect.left() + adjustedLeft * sx,
        targetRect.top() + adjustedTop * sy,
        adjustedWidth * sx,
        adjustedHeight * sy
    ).normalized();
}

void HumanBoxOverlayWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    if (!m_visible || m_boxes.isEmpty()) return;

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, false);
    const QColor boxColor(120, 255, 120);
    QPen boxPen(boxColor, 1.6);
    boxPen.setCosmetic(true);
    p.setBrush(Qt::NoBrush);
    for (const QRect& box : m_boxes) {
        if (!box.isValid()) continue;
        const QRectF mapped = mapRect(box);

        const QString badgeText = "person";
        QFont badgeFont = p.font();
        badgeFont.setPointSize(7);
        badgeFont.setBold(false);
        p.setFont(badgeFont);

        const QFontMetrics fm(badgeFont);
        const int padX = 6;
        const int padY = 2;
        const int badgeW = fm.horizontalAdvance(badgeText) + padX * 2;
        const int badgeH = fm.height() + padY;
        QRectF badgeRect(mapped.left(), std::max(0.0, mapped.top() - badgeH - 1.0), badgeW, badgeH);
        if (badgeRect.top() <= 2.0) {
            badgeRect.moveTop(mapped.top() + 1.0);
        }

        p.setPen(Qt::NoPen);
        QColor badgeColor = boxColor;
        badgeColor.setAlpha(170);
        p.setBrush(badgeColor);
        p.drawRect(badgeRect);
        p.setBrush(Qt::NoBrush);
        p.setPen(QColor(18, 26, 20));
        p.drawText(badgeRect, Qt::AlignCenter, badgeText);

        p.setPen(boxPen);
        p.drawRect(mapped);
    }
}
