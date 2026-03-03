#ifndef MINIMAPWIDGET_H
#define MINIMAPWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QMap>

struct MapNode
{
    QString id;
    QPointF pos;   // world coord (meter)
};

struct MapEdge
{
    QString from;
    QString to;
};

class MiniMapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MiniMapWidget(QWidget *parent = nullptr);

    void setNodes(const QVector<MapNode>& nodes);
    void setEdges(const QVector<MapEdge>& edges);
    void setPolyline(const QVector<QPointF>& path);

    void setRobotPose(const QPointF& pos, double yawRad);
    //void paintEvent(QPaintEvent*);
    void setWorldSize(double widthMeter, double heightMeter);
    void setGoalMarkerVisible(bool visible);
    void setGoalMarker(const QPointF& worldPos);
    void setDrawMapGeometry(bool enabled);

signals:
    void nodeClicked(QString id);
    void worldClicked(QPointF worldPos);
    void nodeMoved(QString id, QPointF worldPos);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    QPointF worldToWidget(const QPointF& w) const;
    QPointF widgetToWorld(const QPointF& p) const;


private:
    QVector<MapNode> m_nodes;
    QVector<MapEdge> m_edges;
    QVector<QPointF> m_polyline;

    QPointF m_robotPos;
    double  m_robotYaw;

    double m_worldWidth;   // meter
    double m_worldHeight;  // meter
    int m_dragNodeIndex = -1;
    bool m_dragging = false;
    bool m_goalVisible = false;
    QPointF m_goalWorld;
    bool m_drawMapGeometry = true;
};

#endif
