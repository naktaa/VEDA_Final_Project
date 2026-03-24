#include "RobotStatusWindow.h"

#include "FramelessHelper.h"
#include "TitleBarWidget.h"

#include <QEvent>
#include <QFrame>
#include <QGraphicsDropShadowEffect>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>

#include <algorithm>

namespace {
class SparklineWidget : public QWidget
{
public:
    explicit SparklineWidget(const QColor& color, QWidget* parent = nullptr)
        : QWidget(parent), m_color(color)
    {
        setMinimumHeight(104);
    }

    void setValues(const QVector<double>& values)
    {
        m_values = values;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), QColor("#192133"));

        painter.setPen(QPen(QColor("#2e4b5d"), 1));
        for (int i = 1; i <= 4; ++i) {
            const int y = rect().top() + (rect().height() * i) / 5;
            painter.drawLine(rect().left() + 8, y, rect().right() - 8, y);
        }

        if (m_values.size() < 2) return;

        const QRectF area = rect().adjusted(10, 10, -10, -12);
        QPainterPath linePath;
        for (int i = 0; i < m_values.size(); ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(m_values.size() - 1);
            const double x = area.left() + (area.width() * t);
            const double clamped = std::clamp(m_values.at(i), 0.0, 100.0);
            const double y = area.bottom() - (clamped / 100.0) * area.height();
            if (i == 0) linePath.moveTo(x, y);
            else linePath.lineTo(x, y);
        }

        QPainterPath fillPath = linePath;
        fillPath.lineTo(area.right(), area.bottom());
        fillPath.lineTo(area.left(), area.bottom());
        fillPath.closeSubpath();

        QColor glow = m_color;
        glow.setAlpha(95);
        painter.fillPath(fillPath, glow);
        painter.setPen(QPen(m_color, 2.0));
        painter.drawPath(linePath);
    }

private:
    QColor m_color;
    QVector<double> m_values;
};

QFrame* makeCard(const QString& title, QWidget* parent)
{
    auto* card = new QFrame(parent);
    card->setObjectName("robotDashboardCard");
    auto* layout = new QVBoxLayout(card);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(10);
    auto* titleLabel = new QLabel(title, card);
    titleLabel->setProperty("robotDashboardTitle", true);
    layout->addWidget(titleLabel);
    return card;
}

QHBoxLayout* appendMetricRow(QVBoxLayout* layout, const QString& labelText, QLabel*& valueLabel)
{
    auto* row = new QHBoxLayout();
    auto* label = new QLabel(labelText);
    label->setProperty("robotMetricLabel", true);
    valueLabel = new QLabel("-");
    valueLabel->setProperty("robotMetricValue", true);
    row->addWidget(label);
    row->addStretch();
    row->addWidget(valueLabel);
    layout->addLayout(row);
    return row;
}

void addShadow(QWidget* widget)
{
    if (!widget) return;
    auto* shadow = new QGraphicsDropShadowEffect(widget);
    shadow->setBlurRadius(24);
    shadow->setOffset(0, 8);
    shadow->setColor(QColor(10, 18, 34, 90));
    widget->setGraphicsEffect(shadow);
}
}

RobotStatusWindow::RobotStatusWindow(QWidget* parent)
    : QWidget(parent)
{
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_DeleteOnClose, false);
    resize(1180, 760);
    setObjectName("robotStatusWindow");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    m_titleBar = new TitleBarWidget(this);
    m_titleBar->setTitleText("Tank Operations");
    m_titleBar->setMaximizeVisible(true);
    m_titleBar->setMinimizeVisible(true);
    root->addWidget(m_titleBar, 0);

    auto* body = new QWidget(this);
    body->setObjectName("robotStatusBody");
    auto* bodyLayout = new QGridLayout(body);
    bodyLayout->setContentsMargins(18, 18, 18, 18);
    bodyLayout->setHorizontalSpacing(14);
    bodyLayout->setVerticalSpacing(14);

    auto* operationalCard = makeCard("Operation Summary", body);
    auto* operationalLayout = qobject_cast<QVBoxLayout*>(operationalCard->layout());
    appendMetricRow(operationalLayout, "Communication", m_connectionValue);
    appendMetricRow(operationalLayout, "Robot State", m_robotStateValue);
    appendMetricRow(operationalLayout, "Data Period", m_dataPeriodValue);
    appendMetricRow(operationalLayout, "Mode", m_modeValue);
    appendMetricRow(operationalLayout, "Mission", m_missionValue);

    auto* motionCard = makeCard("Robot Move", body);
    auto* motionLayout = qobject_cast<QVBoxLayout*>(motionCard->layout());
    auto* motionPanel = new QFrame(motionCard);
    motionPanel->setObjectName("robotMovePanel");
    auto* motionPanelLayout = new QGridLayout(motionPanel);
    motionPanelLayout->setContentsMargins(12, 12, 12, 12);
    motionPanelLayout->setHorizontalSpacing(12);
    motionPanelLayout->setVerticalSpacing(10);
    auto addMotionValue = [&](const QString& labelText, QLabel*& valueLabel, int row, int col) {
        auto* label = new QLabel(labelText, motionPanel);
        label->setProperty("robotMetricLabel", true);
        valueLabel = new QLabel("-", motionPanel);
        valueLabel->setProperty("robotMetricValueLarge", true);
        motionPanelLayout->addWidget(label, row, col);
        motionPanelLayout->addWidget(valueLabel, row + 1, col);
    };
    addMotionValue("X", m_poseXValue, 0, 0);
    addMotionValue("Y", m_poseYValue, 0, 1);
    addMotionValue("Heading", m_poseHeadingValue, 0, 2);
    addMotionValue("Target", m_targetValue, 2, 0);
    addMotionValue("Speed", m_speedValue, 2, 1);
    addMotionValue("Battery", m_batteryValue, 2, 2);
    motionLayout->addWidget(motionPanel, 1);

    auto* telemetryCard = makeCard("Drive Telemetry", body);
    auto* telemetryLayout = qobject_cast<QVBoxLayout*>(telemetryCard->layout());
    appendMetricRow(telemetryLayout, "Run State", m_runStateValue);
    appendMetricRow(telemetryLayout, "Motor 1", m_motor1Value);
    appendMetricRow(telemetryLayout, "Motor 2", m_motor2Value);

    auto* speedCard = makeCard("Speed Trend", body);
    auto* speedLayout = qobject_cast<QVBoxLayout*>(speedCard->layout());
    m_speedTrendHost = new QFrame(speedCard);
    speedLayout->addWidget(m_speedTrendHost, 1);

    auto* batteryCard = makeCard("Battery Trend", body);
    auto* batteryLayout = qobject_cast<QVBoxLayout*>(batteryCard->layout());
    m_batteryTrendHost = new QFrame(batteryCard);
    batteryLayout->addWidget(m_batteryTrendHost, 1);

    auto* taskCard = makeCard("Task Counters", body);
    auto* taskLayout = qobject_cast<QVBoxLayout*>(taskCard->layout());
    m_taskTrendHost = new QFrame(taskCard);
    taskLayout->addWidget(m_taskTrendHost, 1);

    bodyLayout->addWidget(operationalCard, 0, 0);
    bodyLayout->addWidget(motionCard, 0, 1, 1, 2);
    bodyLayout->addWidget(telemetryCard, 0, 3);
    bodyLayout->addWidget(speedCard, 1, 0, 1, 2);
    bodyLayout->addWidget(batteryCard, 1, 2);
    bodyLayout->addWidget(taskCard, 1, 3);
    bodyLayout->setColumnStretch(0, 2);
    bodyLayout->setColumnStretch(1, 3);
    bodyLayout->setColumnStretch(2, 2);
    bodyLayout->setColumnStretch(3, 2);
    bodyLayout->setRowStretch(1, 1);
    root->addWidget(body, 1);

    addShadow(operationalCard);
    addShadow(motionCard);
    addShadow(telemetryCard);
    addShadow(speedCard);
    addShadow(batteryCard);
    addShadow(taskCard);

    connect(m_titleBar, &TitleBarWidget::minimizeRequested, this, &QWidget::showMinimized);
    connect(m_titleBar, &TitleBarWidget::maximizeRequested, this, &QWidget::showMaximized);
    connect(m_titleBar, &TitleBarWidget::restoreRequested, this, &QWidget::showNormal);
    connect(m_titleBar, &TitleBarWidget::closeRequested, this, &QWidget::close);

    setStyleSheet(R"(
#robotStatusWindow {
    background-color: #0f1726;
}
#robotStatusBody {
    background-color: #122033;
}
QFrame#robotDashboardCard {
    background-color: #223047;
    border: 1px solid #314768;
    border-radius: 10px;
}
QLabel[robotDashboardTitle="true"] {
    color: #eef6ff;
    font-size: 17px;
    font-weight: 800;
}
QLabel[robotMetricLabel="true"] {
    color: #8fa6c8;
    font-size: 12px;
    font-weight: 600;
}
QLabel[robotMetricValue="true"] {
    color: #eef6ff;
    font-size: 14px;
    font-weight: 700;
}
QLabel[robotMetricValueLarge="true"] {
    color: #54dbcf;
    font-size: 20px;
    font-weight: 800;
}
QFrame#robotMovePanel {
    background-color: #18253a;
    border: 1px solid #2d435c;
    border-radius: 10px;
}
)");

    updateSparkline(m_speedTrendHost, {0, 0}, QColor("#54dbcf"));
    updateSparkline(m_batteryTrendHost, {0, 0}, QColor("#7ee081"));
    updateSparkline(m_taskTrendHost, {0, 0}, QColor("#f0b54a"));
    syncWindowState();
}

void RobotStatusWindow::setRobotStatus(const MqttEvent& ev)
{
    if (m_connectionValue) {
        const QString commState = ev.rcCommState.isEmpty()
            ? (ev.rcConnected ? "CONNECTED" : "DISCONNECTED")
            : ev.rcCommState.toUpper();
        m_connectionValue->setText(commState);
    }
    if (m_robotStateValue) {
        const QString robotState = ev.rcRobotState.isEmpty()
            ? (ev.rcConnected ? "RUN" : "IDLE")
            : ev.rcRobotState.toUpper();
        m_robotStateValue->setText(robotState);
    }
    if (m_dataPeriodValue) m_dataPeriodValue->setText(ev.rcDataPeriod.isEmpty() ? "-" : ev.rcDataPeriod);
    if (m_modeValue) m_modeValue->setText(ev.rcMode.isEmpty() ? "-" : ev.rcMode.toUpper());
    if (m_missionValue) m_missionValue->setText(ev.rcMission.isEmpty() ? "-" : ev.rcMission);

    if (m_poseXValue) m_poseXValue->setText(QString::number(ev.rcX, 'f', 1));
    if (m_poseYValue) m_poseYValue->setText(QString::number(ev.rcY, 'f', 1));
    if (m_poseHeadingValue) m_poseHeadingValue->setText(QString("%1 deg").arg(ev.rcHeading, 0, 'f', 0));
    if (m_targetValue) {
        m_targetValue->setText(QString("%1, %2")
            .arg(ev.rcTargetX, 0, 'f', 2)
            .arg(ev.rcTargetY, 0, 'f', 2));
    }
    if (m_speedValue) m_speedValue->setText(QString("%1 m/s").arg(ev.rcSpeed, 0, 'f', 2));
    if (m_batteryValue) m_batteryValue->setText(QString("%1 %").arg(ev.rcBattery, 0, 'f', 0));

    if (m_runStateValue) {
        const QString runState = ev.state ? "ACTIVE" : "STANDBY";
        m_runStateValue->setText(runState);
    }

    const auto torqueAt = [&](int index) -> double {
        return (index < ev.rcMotorTorqueValues.size()) ? ev.rcMotorTorqueValues.at(index) : 0.0;
    };
    if (m_motor1Value) m_motor1Value->setText(QString("%1 Nm").arg(torqueAt(0), 0, 'f', 0));
    if (m_motor2Value) m_motor2Value->setText(QString("%1 Nm").arg(torqueAt(1), 0, 'f', 0));

    appendHistory(m_speedHistory, (ev.rcSpeed / 250.0) * 100.0);
    appendHistory(m_batteryHistory, ev.rcBattery);
    appendHistory(m_taskHistory, std::min(static_cast<double>(ev.rcTaskDaily * 4), 100.0));

    updateSparkline(m_speedTrendHost, m_speedHistory, QColor("#54dbcf"));
    updateSparkline(m_batteryTrendHost, m_batteryHistory, QColor("#7ee081"));
    updateSparkline(m_taskTrendHost, m_taskHistory, QColor("#f0b54a"));
}

bool RobotStatusWindow::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
    return FramelessHelper::handleNativeEvent(this, m_titleBar, eventType, message, result);
}

void RobotStatusWindow::changeEvent(QEvent* event)
{
    QWidget::changeEvent(event);
    if (event && event->type() == QEvent::WindowStateChange) syncWindowState();
}

void RobotStatusWindow::updateSparkline(QFrame* host, const QVector<double>& values, const QColor& color)
{
    if (!host) return;

    auto* layout = qobject_cast<QVBoxLayout*>(host->layout());
    if (!layout) {
        layout = new QVBoxLayout(host);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
    }

    auto* chart = (layout->count() > 0)
        ? static_cast<SparklineWidget*>(layout->itemAt(0)->widget())
        : nullptr;

    if (!chart) {
        chart = new SparklineWidget(color, host);
        layout->addWidget(chart);
    }

    chart->setValues(values);
}

void RobotStatusWindow::appendHistory(QVector<double>& history, double value, int maxSize)
{
    history.push_back(std::clamp(value, 0.0, 100.0));
    while (history.size() > maxSize) history.removeFirst();
}

void RobotStatusWindow::syncWindowState()
{
    if (m_titleBar) m_titleBar->setMaximized(isMaximized());
}
