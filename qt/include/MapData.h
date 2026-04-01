#ifndef MAPDATA_H
#define MAPDATA_H

#include <QVector>
#include <QPointF>
#include <QString>

struct MapNodeData {
    QString id;
    QPointF pos;
};

struct MapEdgeData {
    QString from;
    QString to;
};

struct MapData {
    QVector<MapNodeData> nodes;
    QVector<MapEdgeData> edges;
    QVector<QPointF>     polyline;
};

#endif
