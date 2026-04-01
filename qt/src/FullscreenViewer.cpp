#include "FullscreenViewer.h"
#include "GstVideoWidget.h"
#include "CctvZoomWindow.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QEvent>
#include <QMouseEvent>
#include <QCloseEvent>
#include <QWheelEvent>

FullscreenViewer::FullscreenViewer(QWidget* parent)
    : QDialog(parent)
{
    setWindowFlag(Qt::Window);
    setModal(false);

    m_title = new QLabel("Full Screen", this);
    m_title->setStyleSheet("font-weight:700; padding:6px; color:white; background:#111;");

    m_video = new GstVideoWidget(this);
    m_video->setStyleSheet("background:black;");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    root->addWidget(m_title);
    root->addWidget(m_video, 1);
    setLayout(root);

    // ✅ 더블클릭 이벤트는 Dialog 전체에서 받자
    installEventFilter(this);

    //m_zoom = new CctvZoomWindow(nullptr); // 독립창
}

FullscreenViewer::~FullscreenViewer()
{
    stop();
    //delete m_zoom;
}

void FullscreenViewer::openStreamFull(const QString& title, const QString& url)
{
    m_titleText = title;
    m_url = url;

    m_title->setText(title);
    setWindowTitle(title);

    showFullScreen();

    // ✅ show 이후 start (winId 타이밍 안정)
    start(url);
}

void FullscreenViewer::start(const QString& url)
{
    const QString u = url.trimmed();
    if (u.isEmpty()) return;

    if (m_playingUrl == u) return;

    if (m_video) {
        m_video->stopStream();
        m_video->startStream(u);
        m_playingUrl = u;
    }
}

void FullscreenViewer::stop()
{
    if (m_video) m_video->stopStream();
    m_playingUrl.clear();
}

// bool FullscreenViewer::eventFilter(QObject* watched, QEvent* event)
// {
//     Q_UNUSED(watched);

//     // 더블클릭 = 줌 토글
//     if (event->type() == QEvent::MouseButtonDblClick) {
//         static bool zoomed = false;
//         zoomed = !zoomed;

//         if (m_video) {
//             if (zoomed) m_video->setDigitalZoom(2.0, 0.5, 0.5);  // 가운데 2배
//             else        m_video->resetZoom();
//         }
//         return true;
//     }

//     // (선택) 휠로 줌 단계 조절하고 싶으면
//     if (event->type() == QEvent::Wheel) {
//         auto* we = static_cast<QWheelEvent*>(event);
//         const int delta = we->angleDelta().y();

//         if (m_video) {
//             double z = (delta > 0) ? 1.2 : (1.0/1.2); // 20% step
//             // 현재값을 FullscreenViewer가 저장해도 되고, 간단히 여기서만 토글도 가능
//             // 여기선 “간단” 버전: 1.0<->2.0 대신 step 줌이면 FullscreenViewer에 멤버로 zoom 저장 추천
//         }
//         return true;
//     }

//     return QDialog::eventFilter(watched, event);
// }

bool FullscreenViewer::eventFilter(QObject* watched, QEvent* event)
{
    Q_UNUSED(watched);

    if (!m_video) return QDialog::eventFilter(watched, event);

    if (event->type() == QEvent::MouseButtonDblClick) {
        auto* me = static_cast<QMouseEvent*>(event);

        // 1) 더블클릭 좌표를 m_video 로컬 좌표로 변환
        QPoint p = m_video->mapFrom(this, me->pos());

        // 2) 0~1 정규화 (클램프 포함)
        const int w = qMax(1, m_video->width());
        const int h = qMax(1, m_video->height());

        double cx = (double)p.x() / (double)w;
        double cy = (double)p.y() / (double)h;

        if (cx < 0.0) cx = 0.0; if (cx > 1.0) cx = 1.0;
        if (cy < 0.0) cy = 0.0; if (cy > 1.0) cy = 1.0;

        // 3) 줌 토글 (원하면 단계식으로 바꿔도 됨)
        static bool zoomed = false;
        zoomed = !zoomed;

        if (zoomed) m_video->setDigitalZoom(2.0, cx, cy); // ✅ 클릭 지점 기준 확대
        else        m_video->resetZoom();

        return true;
    }

    return QDialog::eventFilter(watched, event);
}

void FullscreenViewer::closeEvent(QCloseEvent* e)
{
    stop();
    QDialog::closeEvent(e);
}
