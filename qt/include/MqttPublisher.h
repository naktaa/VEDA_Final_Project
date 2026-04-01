#ifndef MQTTPUBLISHER_H
#define MQTTPUBLISHER_H
#pragma once

#include <QObject>
#include <QString>
#include <QByteArray>
#include <mosquitto.h>

class MqttPublisher : public QObject {
    Q_OBJECT
public:
    explicit MqttPublisher(QObject* parent=nullptr);
    ~MqttPublisher() override;

    bool start(const QString& host, int port, const QString& clientId="qt-mqtt-pub");
    void stop();

    bool publishJson(const QString& topic, const QByteArray& json, int qos=0, bool retain=false);

signals:
    void logLine(const QString& s);

private:
    QString host_;
    int port_ = 1883;
    mosquitto* mosq_ = nullptr;
    bool started_ = false;
};

#endif // MQTTPUBLISHER_H
