#ifndef CCTVZOOMWINDOW_H
#define CCTVZOOMWINDOW_H

#include <QWidget>
#include <QLabel>
#include <QString>

class GstVideoWidget;

class CctvZoomWindow : public QWidget
{
    Q_OBJECT
public:
    explicit CctvZoomWindow(QWidget* parent = nullptr);
    ~CctvZoomWindow();

    void openStream(const QString& title, const QString& rtspUrl);

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    void start(const QString& rtspUrl);
    void stop();

private:
    QLabel* m_labelTitle = nullptr;
    GstVideoWidget* m_video = nullptr;

    QString m_title;
    QString m_url;

    // ✅ 동일 URL이면 startStream 안 타게 (연타/중복 클릭 검정 줄이기)
    QString m_playingUrl;
};

#endif // CCTVZOOMWINDOW_H


// #ifndef CCTVZOOMWINDOW_H
// #define CCTVZOOMWINDOW_H

// #include <QWidget>
// #include <QLabel>
// #include <QTimer>
// #include <QString>

// #include <opencv2/opencv.hpp>

// class CctvZoomWindow : public QWidget
// {
//     Q_OBJECT
// public:
//     explicit CctvZoomWindow(QWidget* parent = nullptr);
//     ~CctvZoomWindow();

//     void openStream(const QString& title, const QString& rtspUrl);

// protected:
//     void closeEvent(QCloseEvent* e) override;

// private slots:
//     void onTick();

// private:
//     void start(const QString& rtspUrl);
//     void stop();
//     static QImage matToQImage(const cv::Mat& mat);

// private:
//     QLabel* m_labelTitle = nullptr;
//     QLabel* m_labelVideo = nullptr;

//     QTimer* m_timer = nullptr;
//     cv::VideoCapture m_cap;

//     QString m_title;
//     QString m_url;
// };

// #endif // CCTVZOOMWINDOW_H
