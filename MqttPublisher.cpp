#include "MqttPublisher.h"

MqttPublisher::MqttPublisher(QObject* parent) : QObject(parent) {}

MqttPublisher::~MqttPublisher() {
    stop();
}

bool MqttPublisher::start(const QString& host, int port, const QString& clientId)
{
    if (started_) return true;

    host_ = host;
    port_ = port;

    mosquitto_lib_init();
    mosq_ = mosquitto_new(clientId.toStdString().c_str(), true, nullptr);
    if (!mosq_) {
        emit logLine("mosquitto_new failed (publisher)");
        return false;
    }

    int rc = mosquitto_connect(mosq_, host_.toStdString().c_str(), port_, 30);
    if (rc != MOSQ_ERR_SUCCESS) {
        emit logLine(QString("publisher connect failed rc=%1").arg(rc));
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        return false;
    }

    started_ = true;
    emit logLine(QString("publisher connected: %1:%2").arg(host_).arg(port_));
    return true;
}

void MqttPublisher::stop()
{
    started_ = false;

    if (mosq_) {
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
    }
}

bool MqttPublisher::publishJson(const QString& topic, const QByteArray& json, int qos, bool retain)
{
    if (!started_ || !mosq_) {
        emit logLine("publish failed: publisher not started");
        return false;
    }

    int rc = mosquitto_publish(
        mosq_,
        nullptr,
        topic.toStdString().c_str(),
        json.size(),
        json.constData(),
        qos,
        retain
        );

    if (rc != MOSQ_ERR_SUCCESS) {
        emit logLine(QString("publish failed rc=%1 topic=%2").arg(rc).arg(topic));
        // 간단 리커넥트 시도
        mosquitto_reconnect(mosq_);
        return false;
    }

    // loop를 조금 돌려야 실제 전송되는 환경이 있음
    mosquitto_loop(mosq_, 1, 1);
    return true;
}
