#include "MqttSubscriber.h"

#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>

namespace {
double parseTimestampToSeconds(const QJsonValue& value)
{
    if (value.isDouble()) return value.toDouble();

    const QString text = value.toString().trimmed();
    if (text.isEmpty()) return 0.0;

    bool ok = false;
    const double numeric = text.toDouble(&ok);
    if (ok) return numeric;

    const QDateTime dt = QDateTime::fromString(text, Qt::ISODate);
    if (dt.isValid()) return static_cast<double>(dt.toMSecsSinceEpoch()) / 1000.0;

    return 0.0;
}

QRect parsePoseBbox(const QJsonValue& value)
{
    const QJsonArray bbox = value.toArray();
    if (bbox.size() != 4) return {};

    return QRect(
        static_cast<int>(bbox.at(0).toDouble()),
        static_cast<int>(bbox.at(1).toDouble()),
        static_cast<int>(bbox.at(2).toDouble()),
        static_cast<int>(bbox.at(3).toDouble())
    );
}

void parsePoseKeypointMap(const QJsonObject& kpObject, PosePerson& person)
{
    for (auto it = kpObject.begin(); it != kpObject.end(); ++it) {
        const QJsonArray arr = it.value().toArray();
        if (arr.size() < 3) continue;

        PoseKeypoint kp;
        kp.pt = QPointF(arr.at(0).toDouble(), arr.at(1).toDouble());
        kp.visibility = static_cast<float>(arr.at(2).toDouble());
        kp.valid = true;
        person.keypoints.insert(it.key(), kp);
    }
}

void parsePoseKeypointList(const QJsonArray& kpArray, PosePerson& person)
{
    for (const QJsonValue& value : kpArray) {
        const QJsonObject kpObject = value.toObject();
        const QString name = kpObject.value("name").toString();
        if (name.isEmpty()) continue;

        PoseKeypoint kp;
        kp.pt = QPointF(kpObject.value("x").toDouble(), kpObject.value("y").toDouble());
        const QJsonValue visValue = kpObject.contains("score")
                                        ? kpObject.value("score")
                                        : kpObject.value("visibility");
        kp.visibility = static_cast<float>(visValue.toDouble());
        kp.valid = true;
        person.keypoints.insert(name, kp);
    }
}

bool parsePoseFrame(const QJsonObject& root, PoseFrame& out)
{
    out = PoseFrame{};
    out.cameraId = root.value("camera_id").toString();
    if (out.cameraId.isEmpty()) out.cameraId = root.value("sensor_id").toString();
    if (out.cameraId.isEmpty()) out.cameraId = root.value("cam").toString();
    out.frameW = root.value("frame_w").toInt();
    out.frameH = root.value("frame_h").toInt();
    out.ts = parseTimestampToSeconds(root.value("ts"));

    if (root.contains("person")) {
        const QJsonObject personObj = root.value("person").toObject();
        out.person.detected = personObj.value("detected").toBool(false);
        out.person.score = static_cast<float>(personObj.value("score").toDouble(0.0));
        out.person.bbox = parsePoseBbox(personObj.value("bbox"));
        parsePoseKeypointMap(personObj.value("keypoints").toObject(), out.person);
        return true;
    }

    const QString type = root.value("type").toString().trimmed().toLower();

    if (type == "vision_pose") {
        out.person.detected = root.value("person_detected").toBool(false);
        out.person.score = static_cast<float>(root.value("confidence").toDouble(0.0));
        out.person.bbox = parsePoseBbox(root.value("bbox"));
        const QJsonValue keypointsValue = root.value("keypoints");
        if (keypointsValue.isObject()) {
            parsePoseKeypointMap(keypointsValue.toObject(), out.person);
        } else {
            parsePoseKeypointList(keypointsValue.toArray(), out.person);
        }
        return true;
    }

    if (type != "pose") return false;

    const QJsonObject stateObj = root.value("state").toObject();
    out.person.detected = stateObj.value("present").toBool(false);
    out.person.score = static_cast<float>(stateObj.value("confidence").toDouble(0.0));

    const QJsonObject poseObj = root.value("pose").toObject();
    out.person.bbox = parsePoseBbox(poseObj.value("bbox"));
    parsePoseKeypointList(poseObj.value("keypoints").toArray(), out.person);

    const QJsonObject detailsObj = root.value("details").toObject();
    if (out.frameW <= 0) out.frameW = detailsObj.value("frame_w").toInt();
    if (out.frameH <= 0) out.frameH = detailsObj.value("frame_h").toInt();

    return true;
}
}

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

    PoseFrame poseFrame;
    if (parsePoseFrame(o, poseFrame)) {
        emit poseReceived(poseFrame);
    }

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

    if (mqttTopic.contains("/map/graph")) {
        MapData map;

        const QJsonArray nodesA = o.value("nodes").toArray();
        for (const auto& v : nodesA) {
            if (!v.isObject()) continue;
            const QJsonObject n = v.toObject();
            const QString id = n.value("id").toString();
            if (id.isEmpty()) continue;
            map.nodes.push_back({id, QPointF(n.value("x").toDouble(), n.value("y").toDouble())});
        }

        const QJsonArray edgesA = o.value("edges").toArray();
        for (const auto& v : edgesA) {
            if (!v.isObject()) continue;
            const QJsonObject e = v.toObject();
            const QString from = e.value("from").toString();
            const QString to = e.value("to").toString();
            if (from.isEmpty() || to.isEmpty()) continue;
            map.edges.push_back({from, to});
        }

        const QJsonArray polyA = o.value("polyline").toArray();
        for (const auto& v : polyA) {
            if (!v.isObject()) continue;
            const QJsonObject p = v.toObject();
            map.polyline.push_back(QPointF(p.value("x").toDouble(), p.value("y").toDouble()));
        }

        emit mapReceived(map);
        return;
    }

    // event payload
    QString src = "unknown";
    const QStringList parts = mqttTopic.split('/', Qt::SkipEmptyParts);
    if (parts.size() >= 3) src = parts[1];

    const auto jsonCountOrValue = [](const QJsonValue& value) -> int {
        if (value.isArray()) return value.toArray().size();
        if (value.isDouble()) return value.toInt();
        if (value.isString()) {
            bool ok = false;
            const int parsed = value.toString().toInt(&ok);
            return ok ? parsed : 0;
        }
        return 0;
    };

    MqttEvent ev;
    ev.messageType = o.value("type").toString();
    ev.src = o.value("src").toString();
    if (ev.src.isEmpty()) ev.src = o.value("source").toString();

    ev.cam = o.value("cam").toString();
    if (ev.cam.isEmpty()) ev.cam = o.value("zone").toString();

    ev.topic = o.value("topic").toString();
    if (ev.topic.isEmpty()) ev.topic = o.value("event_type").toString();
    if (ev.topic.isEmpty()) ev.topic = o.value("type").toString();
    ev.zone = o.value("zone").toString();
    if (ev.zone.isEmpty() && ev.topic.compare("IvaArea", Qt::CaseInsensitive) == 0) {
        ev.zone = o.value("rule").toString();
    }

    ev.topicFull = o.value("topic_full").toString();
    if (ev.topicFull.isEmpty()) ev.topicFull = mqttTopic;

    ev.utc = o.value("utc").toString();
    if (ev.utc.isEmpty()) ev.utc = o.value("ts").toString();

    ev.rule = o.value("rule").toString();
    if (ev.rule.isEmpty()) {
        const QJsonValue detailsV = o.value("details");
        if (detailsV.isObject()) {
            ev.rule = detailsV.toObject().value("level").toString();
        }
    }

    ev.action = o.value("action").toString();
    if (ev.action.isEmpty()) ev.action = o.value("stateText").toString();

    ev.objectId = o.value("objectId").toString();
    if (ev.objectId.isEmpty()) ev.objectId = o.value("object_id").toString();
    ev.objectType = o.value("objectType").toString();
    ev.sensorId = o.value("sensor_id").toString();
    if (ev.sensorId.isEmpty()) ev.sensorId = o.value("sensorId").toString();
    if (o.contains("bbox_left") && o.contains("bbox_top") &&
        o.contains("bbox_right") && o.contains("bbox_bottom")) {
        const int left = static_cast<int>(o.value("bbox_left").toDouble());
        const int top = static_cast<int>(o.value("bbox_top").toDouble());
        const int right = static_cast<int>(o.value("bbox_right").toDouble());
        const int bottom = static_cast<int>(o.value("bbox_bottom").toDouble());
        ev.bbox = QRect(QPoint(left, top), QPoint(right, bottom)).normalized();
    }
    ev.confidence = o.value("confidence").toDouble(0.0);

    const QJsonValue stateV = o.value("state");
    if (stateV.isObject()) {
        const QJsonObject stateO = stateV.toObject();
        ev.state = stateO.value("present").toBool(false);
        ev.rawState = stateO.value("text").toString().trimmed().toLower();
        if (ev.action.isEmpty()) ev.action = stateO.value("text").toString();
        if (ev.confidence <= 0.0) {
            ev.confidence = stateO.value("confidence").toDouble(0.0);
        }
    } else if (stateV.isBool()) {
        ev.state = stateV.toBool();
    } else if (stateV.isDouble()) {
        ev.state = (stateV.toDouble() != 0.0);
        ev.rawState = QString::number(stateV.toDouble());
    } else {
        ev.rawState = stateV.toString().trimmed().toLower();
        ev.state = (ev.rawState == "true" || ev.rawState == "1" ||
                    ev.rawState == "yes" || ev.rawState == "on" ||
                    ev.rawState == "detected");
    }

    if (!o.contains("state")) {
        if (o.contains("person_detected")) {
            ev.state = o.value("person_detected").toBool(false);
        }
    }

    const QJsonValue enabledV = o.value("enabled");
    if (enabledV.isBool()) {
        ev.enabled = enabledV.toBool();
        ev.hasEnabled = true;
    } else if (enabledV.isDouble()) {
        ev.enabled = (enabledV.toInt() != 0);
        ev.hasEnabled = true;
    } else if (enabledV.isString()) {
        const QString enabledText = enabledV.toString().trimmed().toLower();
        if (!enabledText.isEmpty()) {
            ev.enabled = (enabledText == "true" || enabledText == "1" ||
                          enabledText == "yes" || enabledText == "on" ||
                          enabledText == "enabled");
            ev.hasEnabled = true;
        }
    }

    ev.clipUrl = o.value("clip_url").toString();
    ev.clipSec = o.value("clip_sec").toInt(0);

    const QJsonValue detailsV = o.value("details");
    if (detailsV.isObject()) {
        const QJsonObject d = detailsV.toObject();
        ev.level = d.value("level").toString();
        ev.activeNodes = jsonCountOrValue(d.value("active_nodes"));
        ev.detectedNodes = jsonCountOrValue(d.value("detected_nodes"));
        if (ev.activeNodes <= 0) ev.activeNodes = jsonCountOrValue(d.value("expected"));
        ev.threshold = d.value("threshold").toInt(0);
    }

    if (ev.level.isEmpty()) {
        const QString type = ev.messageType.trimmed().toLower();
        if (type == "vision_pose") {
            ev.level = "pose";
        } else if (type == "coarse_pose") {
            ev.level = "coarse";
        }
    }

    if ((ev.activeNodes <= 0 && ev.detectedNodes <= 0) && o.value("raw_features").isObject()) {
        const QJsonObject raw = o.value("raw_features").toObject();
        ev.activeNodes = jsonCountOrValue(raw.value("active_nodes"));
        ev.detectedNodes = jsonCountOrValue(raw.value("detected_nodes"));
        if (ev.activeNodes <= 0) ev.activeNodes = jsonCountOrValue(raw.value("expected"));
        if (ev.threshold <= 0) ev.threshold = raw.value("threshold").toInt(0);
    }

    if (ev.messageType.compare("rc_status", Qt::CaseInsensitive) == 0 ||
        ev.src.compare("rc", Qt::CaseInsensitive) == 0 ||
        ev.src.compare("rc_car", Qt::CaseInsensitive) == 0) {
        ev.hasRobotStatus = true;
        ev.rcConnected = o.value("connected").toBool(false);
        ev.rcMode = o.value("mode").toString();
        ev.rcMission = o.value("mission").toString();
        ev.rcBattery = o.value("battery").toDouble(0.0);
        ev.rcSpeed = o.value("speed").toDouble(0.0);
        ev.rcX = o.value("x").toDouble(0.0);
        ev.rcY = o.value("y").toDouble(0.0);
        ev.rcHeading = o.value("heading").toDouble(0.0);
        ev.rcZ = o.value("z").toDouble(0.0);
        ev.rcCommState = o.value("comm_state").toString();
        ev.rcRobotState = o.value("robot_state").toString();
        ev.rcDataPeriod = o.value("data_period").toString();
        ev.rcTaskDaily = o.value("task_daily").toInt(0);
        ev.rcTaskWeekly = o.value("task_weekly").toInt(0);
        ev.rcTaskMonthly = o.value("task_monthly").toInt(0);

        const QJsonObject targetObject = o.value("target").toObject();
        if (!targetObject.isEmpty()) {
            ev.rcTargetX = targetObject.value("x").toDouble(0.0);
            ev.rcTargetY = targetObject.value("y").toDouble(0.0);
            ev.rcTargetZ = targetObject.value("z").toDouble(0.0);
        }

        const QJsonArray motors = o.value("motors").toArray();
        for (const QJsonValue& motorValue : motors) {
            const QJsonObject motor = motorValue.toObject();
            ev.rcMotorNames.push_back(motor.value("name").toString());
            ev.rcMotorTorqueValues.push_back(motor.value("torque").toDouble(0.0));
        }
    }

    if (ev.src.isEmpty()) ev.src = src;
    if (ev.cam.isEmpty() && parts.size() >= 4) ev.cam = parts[3];
    if (ev.topic.isEmpty() && parts.size() >= 5) ev.topic = parts[4];
    emit eventReceived(ev);
}
