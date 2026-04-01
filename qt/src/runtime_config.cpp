#include "runtime_config.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace {

QString resolveOptionalPath(const QString& baseDir, const QString& rawValue)
{
    const QString trimmed = rawValue.trimmed();
    if (trimmed.isEmpty()) {
        return {};
    }

    const QFileInfo info(trimmed);
    if (info.isAbsolute()) {
        return QDir::cleanPath(trimmed);
    }

    return QDir::cleanPath(QDir(baseDir).filePath(trimmed));
}

} // namespace

QtRuntimeConfig loadQtRuntimeConfig()
{
    QtRuntimeConfig config;

    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        QDir(appDir).filePath("config/runtime.ini"),
        QDir(appDir).filePath("config/runtime.ini.example"),
        QDir(appDir).filePath("../config/runtime.ini"),
        QDir(appDir).filePath("../config/runtime.ini.example"),
        QDir::current().filePath("config/runtime.ini"),
        QDir::current().filePath("config/runtime.ini.example"),
    };

    QString configPath;
    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) {
            configPath = QDir::cleanPath(candidate);
            break;
        }
    }

    config.configDir = QDir(appDir).filePath("config");
    if (configPath.isEmpty()) {
        return config;
    }

    config.sourcePath = configPath;
    config.configDir = QFileInfo(configPath).absolutePath();

    QSettings settings(configPath, QSettings::IniFormat);
    config.mqttHost = settings.value("mqtt/host", config.mqttHost).toString().trimmed();
    config.mqttPort = settings.value("mqtt/port", config.mqttPort).toInt();
    config.mqttTopicFilter = settings.value("mqtt/topic_filter", config.mqttTopicFilter).toString().trimmed();
    config.mainPublisherClientId =
        settings.value("mqtt/main_publisher_client_id", config.mainPublisherClientId).toString().trimmed();
    config.clipPublisherClientId =
        settings.value("mqtt/clip_publisher_client_id", config.clipPublisherClientId).toString().trimmed();
    config.centralRtsp = settings.value("streams/central_rtsp").toString().trimmed();
    config.tankRtsp = settings.value("streams/tank_rtsp").toString().trimmed();
    config.ruviewZoneConfigPath =
        resolveOptionalPath(config.configDir, settings.value("paths/ruview_zone_config").toString());

    if (config.mqttHost.isEmpty()) {
        config.mqttHost = "localhost";
    }
    if (config.mqttTopicFilter.isEmpty()) {
        config.mqttTopicFilter = "wiserisk/#";
    }
    if (config.mainPublisherClientId.isEmpty()) {
        config.mainPublisherClientId = "qt-main-goal-pub";
    }
    if (config.clipPublisherClientId.isEmpty()) {
        config.clipPublisherClientId = "qt-clip-popup-pub";
    }

    return config;
}
