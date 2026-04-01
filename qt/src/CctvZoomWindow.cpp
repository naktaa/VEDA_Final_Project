#include "CctvZoomWindow.h"
#include "GstVideoWidget.h"

#include <QVBoxLayout>
#include <QCloseEvent>

CctvZoomWindow::CctvZoomWindow(QWidget* parent)
    : QWidget(parent)
{
    setWindowFlags(Qt::Window);
    resize(1100, 650);

    m_labelTitle = new QLabel("Zoom View", this);
    m_labelTitle->setStyleSheet("font-weight:700; padding:6px;");

    // ✅ OpenCV QLabel 대신 GstVideoWidget 사용
    m_video = new GstVideoWidget(this);
    m_video->setStyleSheet("background:#111; border:1px solid #333;");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);
    root->addWidget(m_labelTitle);
    root->addWidget(m_video, 1);
    setLayout(root);

    m_playingUrl.clear();
}

CctvZoomWindow::~CctvZoomWindow()
{
    stop();
}

void CctvZoomWindow::openStream(const QString& title, const QString& rtspUrl)
{
    m_title = title;
    m_url = rtspUrl;

    setWindowTitle(title);
    m_labelTitle->setText(title);

    // ✅ show 먼저: winId 안정화에 도움되는 경우 많음
    show();
    raise();
    activateWindow();

    start(rtspUrl);
}

void CctvZoomWindow::start(const QString& rtspUrl)
{
    const QString url = rtspUrl.trimmed();
    if (url.isEmpty()) {
        // 빈 URL이면 stop하고 끝
        stop();
        return;
    }

    if (!m_video) return;

    // ✅ 동일 URL이면 재시작 금지
    if (m_playingUrl == url) return;

    // ✅ URL 바뀌면 stop/start
    m_video->stopStream();
    m_video->startStream(url);

    m_playingUrl = url;
}

void CctvZoomWindow::stop()
{
    if (m_video) m_video->stopStream();
    m_playingUrl.clear();
}

void CctvZoomWindow::closeEvent(QCloseEvent* e)
{
    stop();
    QWidget::closeEvent(e);
}


// #include "CctvZoomWindow.h"

// #include <QVBoxLayout>
// #include <QCloseEvent>
// #include <QPixmap>

// CctvZoomWindow::CctvZoomWindow(QWidget* parent)
//     : QWidget(parent)
// {
//     setWindowFlags(Qt::Window);
//     resize(1100, 650);

//     m_labelTitle = new QLabel("Zoom View", this);
//     m_labelTitle->setStyleSheet("font-weight:700; padding:6px;");

//     m_labelVideo = new QLabel(this);
//     m_labelVideo->setAlignment(Qt::AlignCenter);
//     m_labelVideo->setStyleSheet("background:#111; border:1px solid #333; color:#9aa0a6;");
//     m_labelVideo->setText("No stream");

//     auto* root = new QVBoxLayout(this);
//     root->setContentsMargins(10, 10, 10, 10);
//     root->addWidget(m_labelTitle);
//     root->addWidget(m_labelVideo, 1);
//     setLayout(root);

//     m_timer = new QTimer(this);
//     connect(m_timer, &QTimer::timeout, this, &CctvZoomWindow::onTick);
// }

// CctvZoomWindow::~CctvZoomWindow()
// {
//     stop();
// }

// void CctvZoomWindow::openStream(const QString& title, const QString& rtspUrl)
// {
//     m_title = title;
//     m_url = rtspUrl;

//     setWindowTitle(title);
//     m_labelTitle->setText(title);

//     start(rtspUrl);

//     show();
//     raise();
//     activateWindow();
// }

// void CctvZoomWindow::start(const QString& rtspUrl)
// {
//     stop();

//     bool ok = m_cap.open(rtspUrl.toStdString());
//     if (!ok || !m_cap.isOpened()) {
//         m_labelVideo->setText("Stream open failed\n" + rtspUrl);
//         return;
//     }

//     m_timer->start(33);
// }

// void CctvZoomWindow::stop()
// {
//     if (m_timer && m_timer->isActive())
//         m_timer->stop();

//     if (m_cap.isOpened())
//         m_cap.release();
// }

// void CctvZoomWindow::closeEvent(QCloseEvent* e)
// {
//     stop();
//     QWidget::closeEvent(e);
// }

// void CctvZoomWindow::onTick()
// {
//     if (!m_cap.isOpened()) return;

//     cv::Mat frame;
//     m_cap >> frame;
//     if (frame.empty()) return;

//     QImage img = matToQImage(frame);
//     if (img.isNull()) return;

//     m_labelVideo->setPixmap(
//         QPixmap::fromImage(img).scaled(
//             m_labelVideo->size(),
//             Qt::KeepAspectRatio,
//             Qt::SmoothTransformation
//             )
//         );
// }

// QImage CctvZoomWindow::matToQImage(const cv::Mat& mat)
// {
//     if (mat.empty()) return QImage();

//     if (mat.type() == CV_8UC3) {
//         cv::Mat rgb;
//         cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
//         return QImage(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888).copy();
//     }

//     if (mat.type() == CV_8UC1) {
//         return QImage(mat.data, mat.cols, mat.rows, (int)mat.step, QImage::Format_Grayscale8).copy();
//     }

//     return QImage();
// }
