#include "CctvOverlayWidget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QtMath>

CctvOverlayWidget::CctvOverlayWidget(QWidget *parent)
    : QWidget(parent),
    m_robotYaw(0.0)
{
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_TranslucentBackground);
}

void CctvOverlayWidget::setHomography(const QTransform& H)
{
    m_H = H;
    update();
}

void CctvOverlayWidget::setNodes(const QVector<OverlayNode>& nodes)
{
    m_nodes = nodes;
    update();
}

void CctvOverlayWidget::setPolyline(const QVector<QPointF>& path)
{
    m_polyline = path;
    update();
}

void CctvOverlayWidget::setMapVisible(bool visible)
{
    m_mapVisible = visible;
    update();
}

void CctvOverlayWidget::setGoalMarkerVisible(bool visible)
{
    m_goalVisible = visible;
    update();
}

void CctvOverlayWidget::setGoalMarker(const QPointF& worldPos)
{
    m_goalWorld = worldPos;
    m_goalVisible = true;
    update();
}

void CctvOverlayWidget::setRobotPose(const QPointF& worldPos, double yawRad)
{
    m_robotPos = worldPos;
    m_robotYaw = yawRad;
    m_hasRobotPose = true;
    update();
}

void CctvOverlayWidget::setRoi(const QRectF& normalizedRoi)
{
    m_roiNorm = normalizedRoi;
    update();
}

void CctvOverlayWidget::setCalibrationMode(bool enabled)
{
    m_calibrationMode = enabled;
    setAttribute(Qt::WA_TransparentForMouseEvents, !(m_calibrationMode || m_goalClickMode));
    update();
}

void CctvOverlayWidget::setGoalClickMode(bool enabled)
{
    m_goalClickMode = enabled;
    setAttribute(Qt::WA_TransparentForMouseEvents, !(m_calibrationMode || m_goalClickMode));
    update();
}

QPointF CctvOverlayWidget::worldToImage(const QPointF& world) const
{
    return m_H.map(world);
}

void CctvOverlayWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // =========================
    // ROI (normalized 0~1)
    // =========================
    if (!m_roiNorm.isNull())
    {
        QRectF r(m_roiNorm.x() * width(),
                 m_roiNorm.y() * height(),
                 m_roiNorm.width() * width(),
                 m_roiNorm.height() * height());

        painter.setPen(QPen(Qt::red, 2));
        painter.drawRect(r);
    }

    if (m_mapVisible) {
        // Map polyline
        painter.setPen(QPen(QColor(0, 220, 255, 230), 3));
        for (int i = 0; i + 1 < m_polyline.size(); ++i) {
            painter.drawLine(worldToImage(m_polyline[i]), worldToImage(m_polyline[i + 1]));
        }

        // Map nodes
        painter.setBrush(Qt::green);
        painter.setPen(Qt::NoPen);
        for (const auto& n : m_nodes)
        {
            QPointF p = worldToImage(n.worldPos);
            painter.drawEllipse(p, 6, 6);
        }
    }

    if (m_hasRobotPose) {
        // =========================
        // Robot
        // =========================
        const QPointF r = worldToImage(m_robotPos);

        painter.setBrush(Qt::red);
        painter.drawEllipse(r, 8, 8);

        const QPointF dir(r.x() + 25 * cos(m_robotYaw),
                          r.y() - 25 * sin(m_robotYaw));

        painter.setPen(QPen(Qt::red, 3));
        painter.drawLine(r, dir);
    }

    if (m_goalVisible) {
        const QPointF g = worldToImage(m_goalWorld);
        painter.setPen(QPen(QColor(255, 255, 255, 240), 2));
        painter.setBrush(QColor(255, 70, 70, 220));
        painter.drawEllipse(g, 9, 9);
    }
}

void CctvOverlayWidget::mousePressEvent(QMouseEvent *event)
{
    if (m_calibrationMode && event) {
    #if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        emit calibrationPressed(event->position());
    #else
        emit calibrationPressed(event->localPos());
    #endif
        event->accept();
        return;
    }
    if (m_goalClickMode && event) {
    #if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        emit goalClicked(event->position());
    #else
        emit goalClicked(event->localPos());
    #endif
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void CctvOverlayWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_calibrationMode && event) {
    #if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        emit calibrationMoved(event->position());
    #else
        emit calibrationMoved(event->localPos());
    #endif
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void CctvOverlayWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_calibrationMode && event) {
    #if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        emit calibrationReleased(event->position());
    #else
        emit calibrationReleased(event->localPos());
    #endif
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}
