#ifndef SYSTEMUSAGEWINDOW_H
#define SYSTEMUSAGEWINDOW_H

#include <QVector>
#include <QWidget>

class QLabel;
class QEvent;
class TitleBarWidget;

class SystemUsageWindow : public QWidget
{
    Q_OBJECT

public:
    explicit SystemUsageWindow(QWidget* parent = nullptr);

    void setUsageData(double cpuPercent,
                      double memoryPercent,
                      double gpuPercent,
                      const QVector<double>& cpuHistory,
                      const QVector<double>& memoryHistory,
                      const QVector<double>& gpuHistory);

protected:
    bool nativeEvent(const QByteArray& eventType, void* message, qintptr* result) override;
    void changeEvent(QEvent* event) override;

private:
    void syncWindowState();

private:
    TitleBarWidget* m_titleBar = nullptr;
    QLabel* m_cpuValueLabel = nullptr;
    QLabel* m_memoryValueLabel = nullptr;
    QLabel* m_gpuValueLabel = nullptr;
    QWidget* m_cpuChart = nullptr;
    QWidget* m_memoryChart = nullptr;
    QWidget* m_gpuChart = nullptr;
};

#endif
