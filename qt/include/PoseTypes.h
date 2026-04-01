#ifndef POSETYPES_H
#define POSETYPES_H

#include <QMap>
#include <QPointF>
#include <QRect>
#include <QString>

struct PoseKeypoint
{
    QPointF pt;
    float visibility = 0.0f;
    bool valid = false;
};

struct PosePerson
{
    bool detected = false;
    float score = 0.0f;
    QRect bbox;
    QMap<QString, PoseKeypoint> keypoints;
};

struct PoseFrame
{
    QString cameraId;
    int frameW = 0;
    int frameH = 0;
    double ts = 0.0;
    PosePerson person;
};

#endif
