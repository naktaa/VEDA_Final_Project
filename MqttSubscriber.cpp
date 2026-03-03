#include "MqttSubscriber.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>

MqttSubscriber::MqttSubscriber(QObject* parent) : QObject(parent) {}

MqttSubscriber::~MqttSubscriber() {
    stop();
}

void MqttSubscriber::start(const QString& host, int port, const QString& topic) {
    if (running_) return;

    host_ = host;
    port_ = port;
    topic_ = topic;

    running_ = true;

    workerThread_ = new QThread();
    this->moveToThread(workerThread_);

    connect(workerThread_, &QThread::started, this, &MqttSubscriber::worker);
    connect(workerThread_, &QThread::finished, workerThread_, &QObject::deleteLater);

    workerThread_->start();
}

void MqttSubscriber::stop() {
    running_ = false;

    if (mosq_) {
        mosquitto_disconnect(mosq_);
    }

    if (workerThread_) {
        workerThread_->quit();
        workerThread_->wait();
        workerThread_ = nullptr;
    }

    if (mosq_) {
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
    }
}

void MqttSubscriber::worker() {
    mosquitto_lib_init();

    mosq_ = mosquitto_new("qt-mqtt-subscriber", true, this);
    if (!mosq_) {
        emit logLine("mosquitto_new failed");
        return;
    }

    mosquitto_message_callback_set(mosq_, &MqttSubscriber::on_message_static);

    int rc = mosquitto_connect(mosq_, host_.toStdString().c_str(), port_, 30);
    if (rc != MOSQ_ERR_SUCCESS) {
        emit logLine(QString("connect failed rc=%1").arg(rc));
        return;
    }

    rc = mosquitto_subscribe(mosq_, nullptr, topic_.toStdString().c_str(), 0);
    if (rc != MOSQ_ERR_SUCCESS) {
        emit logLine(QString("subscribe failed rc=%1").arg(rc));
        return;
    }

    emit logLine("MQTT subscribed: " + topic_);

    while (running_) {
        rc = mosquitto_loop(mosq_, 100, 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            emit logLine(QString("loop error rc=%1 -> reconnect").arg(rc));
            mosquitto_reconnect(mosq_);
        }
    }
}

void MqttSubscriber::on_message_static(mosquitto*, void* obj, const mosquitto_message* msg) {
    auto* self = static_cast<MqttSubscriber*>(obj);
    if (!self || !msg || !msg->payload || !msg->topic) return;

    const QByteArray payload(reinterpret_cast<const char*>(msg->payload), msg->payloadlen);
    const QString topicStr = QString::fromUtf8(msg->topic);
    self->handleMessage(topicStr, payload);
}

void MqttSubscriber::handleMessage(const QString& mqttTopic, const QByteArray& payload)
{
    QJsonParseError err{};
    const QJsonDocument doc = QJsonDocument::fromJson(payload, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        emit logLine("JSON parse error: " + err.errorString());
        return;
    }

    const QJsonObject o = doc.object();

    // RPi publisher payload:
    // {"type":"homography","key":"H_img2world","rows":3,"cols":3,"data":[9 values]}
    if (o.value("type").toString() == "homography") {
        const QJsonArray data = o.value("data").toArray();
        const int rows = o.value("rows").toInt();
        const int cols = o.value("cols").toInt();
        if (rows != 3 || cols != 3 || data.size() != 9) {
            emit logLine("homography payload invalid (rows/cols/data)");
            return;
        }

        const double h11 = data.at(0).toDouble();
        const double h12 = data.at(1).toDouble();
        const double h13 = data.at(2).toDouble();
        const double h21 = data.at(3).toDouble();
        const double h22 = data.at(4).toDouble();
        const double h23 = data.at(5).toDouble();
        const double h31 = data.at(6).toDouble();
        const double h32 = data.at(7).toDouble();
        const double h33 = data.at(8).toDouble();

        // Keep matrix convention aligned with HomographyCalib::cvHToQTransform
        const QTransform hImgToWorld(
            h11, h21, h31,
            h12, h22, h32,
            h13, h23, h33
            );

        emit homographyReceived(hImgToWorld);
        return;
    }

    // map payload is intentionally ignored in current Qt flow
    if (mqttTopic.contains("/map")) {
        emit logLine("Map payload ignored");
        return;
    }

    // event payload
    QString src = "unknown";
    const QStringList parts = mqttTopic.split('/', Qt::SkipEmptyParts);
    if (parts.size() >= 3) src = parts[1];

    MqttEvent ev;
    ev.src = o.value("src").toString();
    ev.cam = o.value("cam").toString();
    ev.topic = o.value("topic").toString();
    ev.topicFull = o.value("topic_full").toString();
    ev.utc = o.value("utc").toString();
    ev.rule = o.value("rule").toString();
    ev.action = o.value("action").toString();
    ev.objectId = o.value("objectId").toString();

    if (o.value("state").isBool()) {
        ev.state = o.value("state").toBool();
    } else {
        ev.rawState = o.value("state").toString();
        ev.state = (ev.rawState == "true");
    }

    ev.clipUrl = o.value("clip_url").toString();
    ev.clipSec = o.value("clip_sec").toInt(0);

    if (ev.src.isEmpty()) ev.src = src;
    emit eventReceived(ev);
}
