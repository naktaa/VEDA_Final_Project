#ifndef CCTVOVERLAYWIDGET_H
#define CCTVOVERLAYWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QRectF>

struct OverlayNode
{
    QString id;
    QPointF worldPos;   // meter
};

class CctvOverlayWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CctvOverlayWidget(QWidget *parent = nullptr);

    void setHomography(const QTransform& H);
    void setNodes(const QVector<OverlayNode>& nodes);
    void setPolyline(const QVector<QPointF>& path);
    void setMapVisible(bool visible);
    void setNodesVisible(bool visible);
    void setGoalMarkerVisible(bool visible);
    void setGoalMarker(const QPointF& worldPos);
    void setRobotPose(const QPointF& worldPos, double yawRad);
    void setRoi(const QRectF& normalizedRoi);  // 0~1 normalized
    void setCalibrationMode(bool enabled);
    void setGoalClickMode(bool enabled);

signals:
    void calibrationPressed(const QPointF& widgetPos);
    void calibrationMoved(const QPointF& widgetPos);
    void calibrationReleased(const QPointF& widgetPos);
    void goalClicked(const QPointF& widgetPos);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    QPointF worldToImage(const QPointF& world) const;

private:
    QTransform m_H;  // world → image
    QVector<OverlayNode> m_nodes;
    QVector<QPointF> m_polyline;

    QPointF m_robotPos;
    double  m_robotYaw;
    bool    m_hasRobotPose = false;

    QRectF  m_roiNorm;
    bool    m_mapVisible = false;
    bool    m_nodesVisible = false;
    bool    m_goalVisible = false;
    QPointF m_goalWorld;
    bool    m_calibrationMode = false;
    bool    m_goalClickMode = false;
};

#endif
