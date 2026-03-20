#ifndef MQTTEVENT_H
#define MQTTEVENT_H
#pragma once
#include <QRect>
#include <QString>
#include <QStringList>
#include <QVector>

struct MqttEvent {
    QString messageType;
    QString src;      // "cctv" or "rpi"
    QString cam;
    QString topic;
    QString topicFull;
    QString utc;
    QString zone;
    QString rule;
    QString action;
    QString objectId;
    QString objectType;
    QString sensorId;
    QRect   bbox;
    int     frameW = 0;
    int     frameH = 0;
    bool    state = false;
    QString rawState;
    bool    enabled = false;
    bool    hasEnabled = false;
    double  confidence = 0.0;
    QString level;
    int     activeNodes = 0;
    int     detectedNodes = 0;
    int     threshold = 0;

    QString clipUrl;
    int     clipSec = 0;

    bool    hasRobotStatus = false;
    bool    rcConnected = false;
    QString rcMode;
    QString rcMission;
    double  rcBattery = 0.0;
    double  rcSpeed = 0.0;
    double  rcX = 0.0;
    double  rcY = 0.0;
    double  rcHeading = 0.0;
    double  rcZ = 0.0;
    double  rcTargetX = 0.0;
    double  rcTargetY = 0.0;
    double  rcTargetZ = 0.0;
    QString rcCommState;
    QString rcRobotState;
    QString rcDataPeriod;
    int     rcTaskDaily = 0;
    int     rcTaskWeekly = 0;
    int     rcTaskMonthly = 0;
    QStringList rcMotorNames;
    QVector<double> rcMotorTorqueValues;
};

#endif // MQTTEVENT_H
