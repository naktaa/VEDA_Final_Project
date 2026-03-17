#ifndef MQTTSUBSCRIBER_H
#define MQTTSUBSCRIBER_H
#pragma once
#include <QObject>
#include <QThread>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTransform>
#include <atomic>
#include <mosquitto.h>
#include "MapData.h"
#include "MqttEvent.h"
#include "PoseTypes.h"
class MqttSubscriber : public QObject {
    Q_OBJECT
public:
    explicit MqttSubscriber(QObject* parent=nullptr);
    ~MqttSubscriber() override;

    void start(const QString& host, int port, const QString& topic);
    void stop();

signals:
    void eventReceived(const MqttEvent& ev);
    void poseReceived(const PoseFrame& frame);
    void mapReceived(const MapData& map);
    void homographyReceived(const QTransform& hImgToWorld);
    void logLine(const QString& s);

private slots:
    void worker();

private:
    static void on_message_static(mosquitto*, void* obj, const mosquitto_message* msg);
    void handleMessage(const QString& mqttTopic, const QByteArray& payload);
private:
    QString host_;
    int port_ = 1883; // 83: cctv , 81: tracingcar
    QString topic_;

    mosquitto* mosq_ = nullptr;
    QThread* workerThread_ = nullptr;
    std::atomic<bool> running_{false};
};

#endif // MQTTSUBSCRIBER_H
