#include "SystemUsageWindow.h"

#include "FramelessHelper.h"
#include "TitleBarWidget.h"

#include <QEvent>
#include <QFrame>
#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QPen>

#include <algorithm>

namespace {
class UsageChartWidget : public QWidget
{
public:
    explicit UsageChartWidget(const QColor& lineColor, QWidget* parent = nullptr)
        : QWidget(parent), m_lineColor(lineColor)
    {
        setMinimumHeight(110);
    }

    void setValues(const QVector<double>& values)
    {
        m_values = values;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.fillRect(rect(), QColor("#151c2b"));

        const QRectF area = rect().adjusted(10, 10, -10, -10);
        p.setPen(QPen(QColor("#273146"), 1));
        for (int i = 1; i <= 4; ++i) {
            const qreal y = area.top() + (area.height() * i) / 5.0;
            p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
        }

        if (m_values.size() < 2) return;

        QPainterPath path;
        for (int i = 0; i < m_values.size(); ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(m_values.size() - 1);
            const qreal x = area.left() + (area.width() * t);
            const double clamped = std::clamp(m_values.at(i), 0.0, 100.0);
            const qreal y = area.bottom() - (clamped / 100.0) * area.height();
            if (i == 0) path.moveTo(x, y);
            else path.lineTo(x, y);
        }

        QPainterPath fillPath = path;
        fillPath.lineTo(area.right(), area.bottom());
        fillPath.lineTo(area.left(), area.bottom());
        fillPath.closeSubpath();

        QColor fillColor = m_lineColor;
        fillColor.setAlpha(55);
        p.fillPath(fillPath, fillColor);
        p.setPen(QPen(m_lineColor, 2.0));
        p.drawPath(path);
    }

private:
    QVector<double> m_values;
    QColor m_lineColor;
};

void addShadow(QWidget* widget)
{
    if (!widget) return;
    auto* shadow = new QGraphicsDropShadowEffect(widget);
    shadow->setBlurRadius(24);
    shadow->setOffset(0, 8);
    shadow->setColor(QColor(8, 12, 24, 90));
    widget->setGraphicsEffect(shadow);
}

QFrame* makeMetricCard(const QString& title, const QColor& accent, QLabel*& valueLabel, QWidget*& chartWidget)
{
    auto* card = new QFrame();
    card->setObjectName("usageCard");
    auto* layout = new QVBoxLayout(card);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(10);

    auto* header = new QHBoxLayout();
    auto* titleLabel = new QLabel(title, card);
    titleLabel->setObjectName("usageTitle");
    valueLabel = new QLabel("- %", card);
    valueLabel->setObjectName("usageValue");
    header->addWidget(titleLabel);
    header->addStretch();
    header->addWidget(valueLabel);

    auto* chart = new UsageChartWidget(accent, card);
    chartWidget = chart;

    layout->addLayout(header);
    layout->addWidget(chart);
    addShadow(card);
    return card;
}
}

SystemUsageWindow::SystemUsageWindow(QWidget* parent)
    : QWidget(parent)
{
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_DeleteOnClose, false);
    resize(420, 620);
    setObjectName("systemUsageWindow");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    m_titleBar = new TitleBarWidget(this);
    m_titleBar->setTitleText("System Usage Monitor");
    m_titleBar->setMaximizeVisible(true);
    m_titleBar->setMinimizeVisible(true);
    root->addWidget(m_titleBar, 0);

    auto* body = new QWidget(this);
    body->setObjectName("usageBody");
    auto* bodyLayout = new QVBoxLayout(body);
    bodyLayout->setContentsMargins(18, 18, 18, 18);
    bodyLayout->setSpacing(14);

    bodyLayout->addWidget(makeMetricCard("CPU", QColor("#2ee6a5"), m_cpuValueLabel, m_cpuChart));
    bodyLayout->addWidget(makeMetricCard("Memory", QColor("#53a7ff"), m_memoryValueLabel, m_memoryChart));
    bodyLayout->addWidget(makeMetricCard("GPU", QColor("#8a63ff"), m_gpuValueLabel, m_gpuChart));
    root->addWidget(body, 1);

    connect(m_titleBar, &TitleBarWidget::minimizeRequested, this, &QWidget::showMinimized);
    connect(m_titleBar, &TitleBarWidget::maximizeRequested, this, &QWidget::showMaximized);
    connect(m_titleBar, &TitleBarWidget::restoreRequested, this, &QWidget::showNormal);
    connect(m_titleBar, &TitleBarWidget::closeRequested, this, &QWidget::close);

    setStyleSheet(R"(
#systemUsageWindow {
    background-color: #101726;
}
#usageBody {
    background-color: #121a29;
}
QFrame#usageCard {
    background-color: #1d2638;
    border: 1px solid #2f3c56;
    border-radius: 10px;
}
QLabel#usageTitle {
    color: #b5c0da;
    font-size: 12px;
    font-weight: 700;
    letter-spacing: 0.5px;
}
QLabel#usageValue {
    color: #f5f8ff;
    font-size: 18px;
    font-weight: 800;
}
)");

    syncWindowState();
}

void SystemUsageWindow::setUsageData(double cpuPercent,
                                     double memoryPercent,
                                     double gpuPercent,
                                     const QVector<double>& cpuHistory,
                                     const QVector<double>& memoryHistory,
                                     const QVector<double>& gpuHistory)
{
    if (m_cpuValueLabel) m_cpuValueLabel->setText(QString("%1 %").arg(cpuPercent, 0, 'f', 0));
    if (m_memoryValueLabel) m_memoryValueLabel->setText(QString("%1 %").arg(memoryPercent, 0, 'f', 0));
    if (m_gpuValueLabel) {
        m_gpuValueLabel->setText(gpuPercent >= 0.0
            ? QString("%1 %").arg(gpuPercent, 0, 'f', 0)
            : QString("- %"));
    }

    if (auto* chart = static_cast<UsageChartWidget*>(m_cpuChart)) chart->setValues(cpuHistory);
    if (auto* chart = static_cast<UsageChartWidget*>(m_memoryChart)) chart->setValues(memoryHistory);
    if (auto* chart = static_cast<UsageChartWidget*>(m_gpuChart)) chart->setValues(gpuHistory);
}

bool SystemUsageWindow::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
    return FramelessHelper::handleNativeEvent(this, m_titleBar, eventType, message, result);
}

void SystemUsageWindow::changeEvent(QEvent* event)
{
    QWidget::changeEvent(event);
    if (event && event->type() == QEvent::WindowStateChange) syncWindowState();
}

void SystemUsageWindow::syncWindowState()
{
    if (m_titleBar) m_titleBar->setMaximized(isMaximized());
}
