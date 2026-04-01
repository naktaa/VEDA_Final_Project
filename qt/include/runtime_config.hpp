#pragma once

#include <QString>

struct QtRuntimeConfig {
    QString sourcePath;
    QString configDir;
    QString mqttHost = "localhost";
    int mqttPort = 1883;
    QString mqttTopicFilter = "wiserisk/#";
    QString mainPublisherClientId = "qt-main-goal-pub";
    QString clipPublisherClientId = "qt-clip-popup-pub";
    QString centralRtsp;
    QString tankRtsp;
    QString ruviewZoneConfigPath;
};

QtRuntimeConfig loadQtRuntimeConfig();
