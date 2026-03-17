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

    QFrame* m_speedTrendHost = nullptr;
    QFrame* m_batteryTrendHost = nullptr;
    QFrame* m_taskTrendHost = nullptr;

    QVector<double> m_speedHistory;
    QVector<double> m_batteryHistory;
    QVector<double> m_taskHistory;
};

#endif
