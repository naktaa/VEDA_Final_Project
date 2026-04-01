#include "MiniMapWidget.h"

#include <algorithm>
#include <QLineF>
#include <QMouseEvent>
#include <QPainter>

MiniMapWidget::MiniMapWidget(QWidget *parent)
    : QWidget(parent),
      m_robotYaw(0.0),
      m_worldWidth(5.0),
      m_worldHeight(5.0)
{
    setMinimumSize(300, 300);
    setAutoFillBackground(false);
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAttribute(Qt::WA_NoSystemBackground, true);
}

void MiniMapWidget::setNodes(const QVector<MapNode>& nodes)
{
    m_nodes = nodes;
    update();
}

void MiniMapWidget::setEdges(const QVector<MapEdge>& edges)
{
    m_edges = edges;
    update();
}

void MiniMapWidget::setPolyline(const QVector<QPointF>& path)
{
    m_polyline = path;
    update();
}

void MiniMapWidget::setRobotPose(const QPointF& pos, double yawRad)
{
    m_robotPos = pos;
    m_robotYaw = yawRad;
    update();
}

void MiniMapWidget::setWorldSize(double widthMeter, double heightMeter)
{
    m_worldWidth = widthMeter;
    m_worldHeight = heightMeter;
    update();
}

void MiniMapWidget::setGoalMarkerVisible(bool visible)
{
    m_goalVisible = visible;
    update();
}

void MiniMapWidget::setGoalMarker(const QPointF& worldPos)
{
    m_goalWorld = worldPos;
    m_goalVisible = true;
    update();
}

void MiniMapWidget::setDrawMapGeometry(bool enabled)
{
    m_drawMapGeometry = enabled;
    update();
}

void MiniMapWidget::setWorldClickEnabled(bool enabled)
{
    m_worldClickEnabled = enabled;
}

void MiniMapWidget::setZonePresenceState(bool present, double confidence)
{
    m_zonePresent = present;
    m_zoneConfidence = std::max(0.0, std::min(1.0, confidence));
    update();
}

QPointF MiniMapWidget::worldToWidget(const QPointF& w) const
{
    const double sx = width() / m_worldWidth;
    const double sy = height() / m_worldHeight;
    return QPointF(w.x() * sx, height() - w.y() * sy);
}

QPointF MiniMapWidget::widgetToWorld(const QPointF& p) const
{
    const double sx = m_worldWidth / width();
    const double sy = m_worldHeight / height();
    return QPointF(p.x() * sx, (height() - p.y()) * sy);
}

QPointF MiniMapWidget::zoneCenterWorld() const
{
    if (m_polyline.isEmpty()) {
        return QPointF(m_worldWidth * 0.5, m_worldHeight * 0.5);
    }

    double sx = 0.0;
    double sy = 0.0;
    for (const auto& p : m_polyline) {
        sx += p.x();
        sy += p.y();
    }
    const double n = static_cast<double>(m_polyline.size());
    if (n <= 0.0) {
        return QPointF(m_worldWidth * 0.5, m_worldHeight * 0.5);
    }
    return QPointF(sx / n, sy / n);
}

void MiniMapWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    // Clear the layer first to avoid stale translucent artifacts.
    painter.setCompositionMode(QPainter::CompositionMode_Source);
    painter.fillRect(rect(), Qt::transparent);
    painter.setCompositionMode(QPainter::CompositionMode_SourceOver);

    if (m_drawMapGeometry && m_zonePresent && m_polyline.size() >= 3) {
        QPolygonF poly;
        poly.reserve(m_polyline.size());
        for (const auto& p : m_polyline) poly << worldToWidget(p);

        const int alpha = static_cast<int>(60 + (120 * m_zoneConfidence));
        const QColor fill = QColor(255, 70, 70, alpha);

        painter.setPen(Qt::NoPen);
        painter.setBrush(fill);
        painter.drawPolygon(poly);
    }

    if (m_zonePresent) {
        const QPointF c = worldToWidget(zoneCenterWorld());
        const double scale = 18.0 + (8.0 * m_zoneConfidence);

        painter.setPen(QPen(QColor(255, 245, 120, 240), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.setBrush(QColor(255, 245, 120, 220));

        // Approximate inferred human figure for blind-zone presence.
        painter.drawEllipse(QPointF(c.x(), c.y() - scale * 1.6), scale * 0.35, scale * 0.35);
        painter.drawLine(QPointF(c.x(), c.y() - scale * 1.2), QPointF(c.x(), c.y() + scale * 0.3));
        painter.drawLine(QPointF(c.x(), c.y() - scale * 0.6), QPointF(c.x() - scale * 0.7, c.y() - scale * 0.1));
        painter.drawLine(QPointF(c.x(), c.y() - scale * 0.6), QPointF(c.x() + scale * 0.7, c.y() - scale * 0.1));
        painter.drawLine(QPointF(c.x(), c.y() + scale * 0.3), QPointF(c.x() - scale * 0.55, c.y() + scale * 1.2));
        painter.drawLine(QPointF(c.x(), c.y() + scale * 0.3), QPointF(c.x() + scale * 0.55, c.y() + scale * 1.2));
    }

    if (m_drawMapGeometry) {
        // Draw map lines only when geometry overlay is enabled.
        painter.setPen(QPen(QColor(0, 220, 255, 210), 3));
        for (const auto& e : m_edges) {
            QPointF a;
            QPointF b;
            for (const auto& n : m_nodes) {
                if (n.id == e.from) a = n.pos;
                if (n.id == e.to) b = n.pos;
            }
            painter.drawLine(worldToWidget(a), worldToWidget(b));
        }
    }

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(255, 102, 102, 230));
    for (const auto& n : m_nodes) {
        const QPointF p = worldToWidget(n.pos);
        painter.drawEllipse(p, 8, 8);
        painter.setPen(QPen(QColor(255, 255, 255, 230), 1));
        painter.drawText(QRectF(p.x() + 8, p.y() - 16, 28, 20), Qt::AlignLeft | Qt::AlignVCenter, n.id);
        painter.setPen(Qt::NoPen);
    }

    // Polyline is intentionally not drawn here to avoid duplicate boxes on MiniMap.

    if (m_goalVisible) {
        const QPointF p = worldToWidget(m_goalWorld);
        painter.setPen(QPen(QColor(255, 255, 255, 240), 2));
        painter.setBrush(QColor(255, 70, 70, 220));
        painter.drawEllipse(p, 9, 9);
    }
}

void MiniMapWidget::mousePressEvent(QMouseEvent *event)
{
    const QPointF world = widgetToWorld(event->pos());

    for (int i = 0; i < m_nodes.size(); ++i) {
        const auto& n = m_nodes[i];
        if (QLineF(n.pos, world).length() < 0.28) {
            m_dragNodeIndex = i;
            m_dragging = true;
            emit nodeClicked(n.id);
            return;
        }
    }

    if (m_worldClickEnabled) {
        emit worldClicked(world);
    }
}

void MiniMapWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_dragging || m_dragNodeIndex < 0 || m_dragNodeIndex >= m_nodes.size()) {
        QWidget::mouseMoveEvent(event);
        return;
    }

    const QPointF world = widgetToWorld(event->pos());
    m_nodes[m_dragNodeIndex].pos = world;
    emit nodeMoved(m_nodes[m_dragNodeIndex].id, world);
    update();
    event->accept();
}

void MiniMapWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_dragging) {
        if (m_dragNodeIndex >= 0 && m_dragNodeIndex < m_nodes.size()) {
            emit nodeMoved(m_nodes[m_dragNodeIndex].id, m_nodes[m_dragNodeIndex].pos);
        }
        m_dragging = false;
        m_dragNodeIndex = -1;
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}
