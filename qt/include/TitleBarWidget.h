#ifndef TITLEBARWIDGET_H
#define TITLEBARWIDGET_H

#include <QWidget>

class QLabel;
class QPushButton;

class TitleBarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TitleBarWidget(QWidget* parent = nullptr);

    void setTitleText(const QString& text);
    void setMaximizeVisible(bool visible);
    void setMinimizeVisible(bool visible);
    void setMaximized(bool maximized);

signals:
    void minimizeRequested();
    void maximizeRequested();
    void restoreRequested();
    void closeRequested();

private slots:
    void onMaximizeClicked();

private:
    QLabel* m_title = nullptr;
    QPushButton* m_btnMin = nullptr;
    QPushButton* m_btnMax = nullptr;
    QPushButton* m_btnClose = nullptr;
    bool m_isMaximized = false;
};

#endif // TITLEBARWIDGET_H
