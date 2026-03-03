#ifndef MQTTEVENT_H
#define MQTTEVENT_H
#pragma once
#include <QString>

struct MqttEvent {
    QString src;      // "cctv" or "rpi"
    QString cam;
    QString topic;
    QString topicFull;
    QString utc;
    QString rule;
    QString action;
    QString objectId;
    bool    state = false;
    QString rawState;

    QString clipUrl;
    int     clipSec = 0;
};

#endif // MQTTEVENT_H
