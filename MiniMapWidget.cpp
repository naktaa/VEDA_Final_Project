#include "MiniMapWidget.h"

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

void MiniMapWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setCompositionMode(QPainter::CompositionMode_SourceOver);

    if (m_drawMapGeometry) {
        // Only draw map lines with transparent background.
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

    emit worldClicked(world);
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
