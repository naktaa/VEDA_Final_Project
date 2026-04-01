#ifndef ROBOTSTATUSWINDOW_H
#define ROBOTSTATUSWINDOW_H

#include <QByteArray>
#include <QVector>
#include <QWidget>

#include "MqttEvent.h"

class QLabel;
class QFrame;
class QEvent;
class TitleBarWidget;

class RobotStatusWindow : public QWidget
{
    Q_OBJECT

public:
    explicit RobotStatusWindow(QWidget* parent = nullptr);

    void setRobotStatus(const MqttEvent& ev);

protected:
    bool nativeEvent(const QByteArray& eventType, void* message, qintptr* result) override;
    void changeEvent(QEvent* event) override;

private:
    void updateSparkline(QFrame* host, const QVector<double>& values, const QColor& color);
    void appendHistory(QVector<double>& history, double value, int maxSize = 20);
    void applyOperationSummary(const QString& commState,
                               const QString& robotState,
                               const QString& dataPeriod,
                               const QString& mode,
                               const QString& mission);
    void syncWindowState();

    TitleBarWidget* m_titleBar = nullptr;

    QLabel* m_connectionValue = nullptr;
    QLabel* m_robotStateValue = nullptr;
    QLabel* m_dataPeriodValue = nullptr;
    QLabel* m_modeValue = nullptr;
    QLabel* m_missionValue = nullptr;

    QLabel* m_poseXValue = nullptr;
    QLabel* m_poseYValue = nullptr;
    QLabel* m_poseHeadingValue = nullptr;
    QLabel* m_targetValue = nullptr;
    QLabel* m_speedValue = nullptr;
    QLabel* m_batteryValue = nullptr;

    QLabel* m_runStateValue = nullptr;
    QLabel* m_motor1Value = nullptr;
    QLabel* m_motor2Value = nullptr;
    QLabel* m_cpuUsageValue = nullptr;
    QLabel* m_memoryUsageValue = nullptr;

    QFrame* m_speedTrendHost = nullptr;
    QFrame* m_taskTrendHost = nullptr;
    QFrame* m_cpuTrendHost = nullptr;
    QFrame* m_memoryTrendHost = nullptr;

    QVector<double> m_speedHistory;
    QVector<double> m_taskHistory;
    QVector<double> m_cpuUsageHistory;
    QVector<double> m_memoryUsageHistory;

    bool m_manualOperationSummaryLocked = false;
    QString m_lockedCommState;
    QString m_lockedRobotState;
    QString m_lockedDataPeriod;
    QString m_lockedMode;
    QString m_lockedMission;
};

#endif
