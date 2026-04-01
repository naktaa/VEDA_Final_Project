#include "video_clip.h"
#include "TitleBarWidget.h"
#include "FramelessHelper.h"

#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressDialog>

#include <QMediaPlayer>
#include <QAudioOutput>
#include <QVideoWidget>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>

#include <QFile>
#include <QDateTime>
#include <algorithm>
#include <QEvent>
#include <QIcon>
#include <QSize>
#include <QTimer>

namespace {
constexpr int kMediaButtonSize = 36;
constexpr int kMediaIconSize = 20;

void applyPlaybackButtonState(QPushButton* button, bool isPlaying)
{
    if (!button) return;

    const QIcon icon(isPlaying ? QStringLiteral(":/image/stop.png")
                               : QStringLiteral(":/image/play.png"));
    button->setIcon(icon);
    button->setText(icon.isNull() ? (isPlaying ? QStringLiteral("Stop")
                                               : QStringLiteral("Play"))
                                  : QString());
}
}

VideoClipWindow::VideoClipWindow(QWidget* parent)
    : QWidget(parent)
{
    // 독립 창처럼 띄우기
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    resize(960, 540);

    buildUi();

    // player
    m_player = new QMediaPlayer(this);
    m_audio = new QAudioOutput(this);
    m_player->setAudioOutput(m_audio);
    m_player->setVideoOutput(m_video);

    connect(m_player, &QMediaPlayer::positionChanged, this, &VideoClipWindow::onPositionChanged);
    connect(m_player, &QMediaPlayer::durationChanged, this, &VideoClipWindow::onDurationChanged);

    // network
    m_net = new QNetworkAccessManager(this);
}

VideoClipWindow::~VideoClipWindow()
{
    if (m_reply) {
        m_reply->abort();
        m_reply->deleteLater();
        m_reply = nullptr;
    }
}

void VideoClipWindow::buildUi()
{
    m_titleBar = new TitleBarWidget(this);
    m_titleBar->setTitleText("Clip Player");
    m_titleLabel = new QLabel("Clip Player", this);
    m_titleLabel->setStyleSheet("font-weight: 600;");

    m_video = new QVideoWidget(this);

    m_btnPlayPause = new QPushButton(this);
    for (QPushButton* button : {m_btnPlayPause}) {
        button->setFlat(true);
        button->setCursor(Qt::PointingHandCursor);
        button->setIconSize(QSize(kMediaIconSize, kMediaIconSize));
        button->setFixedSize(kMediaButtonSize, kMediaButtonSize);
        button->setStyleSheet(
            "QPushButton { border: none; background: transparent; padding: 0px; border-radius: 8px; }"
            "QPushButton:hover { background: rgba(255,255,255,0.10); }"
            "QPushButton:pressed { background: rgba(255,255,255,0.16); }");
    }
    applyPlaybackButtonState(m_btnPlayPause, false);

    m_slider = new QSlider(Qt::Horizontal, this);
    m_slider->setRange(0, 0);

    m_timeLabel = new QLabel("00:00 / 00:00", this);

    m_btnDownload = new QPushButton("Download", this);
    m_btnClose = new QPushButton("Close", this);

    connect(m_btnPlayPause, &QPushButton::clicked, this, &VideoClipWindow::onPlayPause);
    connect(m_btnDownload, &QPushButton::clicked, this, &VideoClipWindow::onDownload);
    connect(m_btnClose, &QPushButton::clicked, this, &VideoClipWindow::onClose);

    connect(m_slider, &QSlider::sliderPressed, this, &VideoClipWindow::onSliderPressed);
    connect(m_slider, &QSlider::sliderReleased, this, &VideoClipWindow::onSliderReleased);
    connect(m_slider, &QSlider::sliderMoved, this, &VideoClipWindow::onSliderMoved);

    auto* top = new QHBoxLayout();
    top->addWidget(m_titleLabel);
    top->addStretch();

    auto* controls = new QHBoxLayout();
    controls->addWidget(m_btnPlayPause);
    controls->addWidget(m_slider, 1);
    controls->addWidget(m_timeLabel);
    controls->addSpacing(8);
    controls->addWidget(m_btnDownload);
    controls->addWidget(m_btnClose);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->addWidget(m_titleBar);
    root->addLayout(top);
    root->addWidget(m_video, 1);
    root->addLayout(controls);
    setLayout(root);
    setStyleSheet("background-color:#171924;");

    connect(m_titleBar, &TitleBarWidget::minimizeRequested, this, &QWidget::showMinimized);
    connect(m_titleBar, &TitleBarWidget::maximizeRequested, this, [this]() {
        if (!isMaximized()) m_normalGeometry = geometry();
        showMaximized();
        if (m_titleBar) m_titleBar->setMaximized(true);
        syncWindowLayout();
    });
    connect(m_titleBar, &TitleBarWidget::restoreRequested, this, [this]() {
        showNormal();
        if (m_normalGeometry.isValid()) setGeometry(m_normalGeometry);
        if (m_titleBar) m_titleBar->setMaximized(false);
        syncWindowLayout();
    });
    connect(m_titleBar, &TitleBarWidget::closeRequested, this, [this]() {
        if (m_player) m_player->stop();
        close();
    });
}

void VideoClipWindow::syncWindowLayout()
{
    if (layout()) {
        layout()->invalidate();
        layout()->activate();
    }
    if (m_video) {
        m_video->updateGeometry();
        m_video->update();
    }
    updateGeometry();
    update();
    QTimer::singleShot(0, this, [this]() {
        if (layout()) {
            layout()->invalidate();
            layout()->activate();
        }
        if (m_video) {
            m_video->updateGeometry();
            m_video->update();
        }
        updateGeometry();
        update();
    });
}

QUrl VideoClipWindow::chooseUrlByRole(UserRole role, const QUrl& privacyUrl, const QUrl& rawUrl) const
{
    // central은 rawUrl만 주는 형태로 사용 가능 → privacyUrl이 비어도 OK
    if (role == UserRole::Executive) {
        if (rawUrl.isValid() && !rawUrl.isEmpty()) return rawUrl;
        return privacyUrl;
    }
    // 관제실: privacy만
    return privacyUrl;
}

void VideoClipWindow::openClip(const QString& title,
                               UserRole role,
                               const QUrl& privacyUrl,
                               const QUrl& rawUrl)
{
    m_title = title;
    m_role = role;
    m_privacyUrl = privacyUrl;
    m_rawUrl = rawUrl;

    // role 정책 적용
    m_currentUrl = chooseUrlByRole(role, privacyUrl, rawUrl);

    // 관제실인데 privacyUrl이 없으면 재생 불가
    if (role == UserRole::ControlRoom && (!m_currentUrl.isValid() || m_currentUrl.isEmpty())) {
        QMessageBox::warning(this, "Clip", "관제실 권한: 재생 가능한 Privacy clip URL이 없음");
        return;
    }

    m_titleLabel->setText(m_title);
    if (m_titleBar) m_titleBar->setTitleText(m_title);
    setWindowTitle(m_title);

    loadAndPlay(m_currentUrl);

    show();
    raise();
    activateWindow();

    const bool canDownload = (m_currentUrl.isValid()
                              && (m_currentUrl.scheme() == "http" || m_currentUrl.scheme() == "https"));
    m_btnDownload->setEnabled(canDownload);

}

void VideoClipWindow::loadAndPlay(const QUrl& url)
{
    if (!url.isValid() || url.isEmpty()) {
        QMessageBox::warning(this, "Clip", "Clip URL invalid");
        return;
    }

    m_player->stop();
    m_player->setSource(url);
    m_player->play();

    applyPlaybackButtonState(m_btnPlayPause, true);
}

void VideoClipWindow::onPlayPause()
{
    if (!m_player) return;

    if (m_player->playbackState() == QMediaPlayer::PlayingState) {
        m_player->pause();
        applyPlaybackButtonState(m_btnPlayPause, false);
    } else {
        m_player->play();
        applyPlaybackButtonState(m_btnPlayPause, true);
    }
}

void VideoClipWindow::onStop()
{
    if (!m_player) return;
    m_player->stop();
    applyPlaybackButtonState(m_btnPlayPause, false);
}

void VideoClipWindow::onClose()
{
    if (m_player) m_player->stop();
    close();
}

void VideoClipWindow::onPositionChanged(qint64 pos)
{
    if (!m_sliderDragging) {
        m_slider->setValue(static_cast<int>(pos));
    }

    m_timeLabel->setText(QString("%1 / %2")
                             .arg(msToTime(pos), msToTime(m_durationMs)));
}

void VideoClipWindow::onDurationChanged(qint64 dur)
{
    m_durationMs = dur;
    m_slider->setRange(0, static_cast<int>(dur));
    m_timeLabel->setText(QString("00:00 / %1").arg(msToTime(dur)));
}

void VideoClipWindow::onSliderPressed()
{
    m_sliderDragging = true;
}

void VideoClipWindow::onSliderReleased()
{
    m_sliderDragging = false;
    if (m_player) m_player->setPosition(m_slider->value());
}

void VideoClipWindow::onSliderMoved(int value)
{
    // 드래그 중 시간 미리 보기
    if (m_sliderDragging) {
        m_timeLabel->setText(QString("%1 / %2")
                                 .arg(msToTime(value), msToTime(m_durationMs)));
    }
}

QString VideoClipWindow::msToTime(qint64 ms)
{
    qint64 totalSec = ms / 1000;
    qint64 min = totalSec / 60;
    qint64 sec = totalSec % 60;
    return QString("%1:%2")
        .arg(min, 2, 10, QChar('0'))
        .arg(sec, 2, 10, QChar('0'));
}

// ---------------- Download (HTTP mp4) ----------------
void VideoClipWindow::onDownload()
{
    if (!m_currentUrl.isValid() || m_currentUrl.isEmpty()) {
        QMessageBox::warning(this, "Download", "다운로드할 URL이 없음");
        return;
    }

    if (!(m_currentUrl.scheme() == "http" || m_currentUrl.scheme() == "https")) {
        QMessageBox::warning(this, "Download", "HTTP/HTTPS URL만 다운로드 가능");
        return;
    }

    const QString defaultName =
        QString("clip_%1.mp4").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));

    const QString savePath = QFileDialog::getSaveFileName(
        this,
        "Save clip",
        defaultName,
        "MP4 Video (*.mp4);;All Files (*.*)"
        );

    if (savePath.isEmpty()) return;

    // 기존 다운로드 중이면 중단
    if (m_reply) {
        m_reply->abort();
        m_reply->deleteLater();
        m_reply = nullptr;
    }
    if (m_outFile) {
        m_outFile->close();
        m_outFile->deleteLater();
        m_outFile = nullptr;
    }

    m_outFile = new QFile(savePath, this);
    if (!m_outFile->open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "Download", "파일 저장 실패");
        m_outFile->deleteLater();
        m_outFile = nullptr;
        return;
    }

    QNetworkRequest req(m_currentUrl);
    req.setAttribute(QNetworkRequest::RedirectPolicyAttribute,
                     QNetworkRequest::NoLessSafeRedirectPolicy);
    req.setHeader(QNetworkRequest::UserAgentHeader, "QtClipDownloader/1.0");
    req.setRawHeader("Accept", "video/mp4,*/*");

    m_reply = m_net->get(req);

    // 진행 다이얼로그
    if (m_progress) {
        m_progress->deleteLater();
        m_progress = nullptr;
    }
    m_progress = new QProgressDialog("Downloading...", "Cancel", 0, 100, this);
    m_progress->setWindowModality(Qt::WindowModal);
    m_progress->setMinimumDuration(0);

    connect(m_progress, &QProgressDialog::canceled, this, [this]() {
        if (m_reply) m_reply->abort();
    });

    connect(m_reply, &QNetworkReply::downloadProgress,
            this, &VideoClipWindow::onDownloadProgress);

    // ✅ chunk 수신될 때마다 파일에 저장
    connect(m_reply, &QNetworkReply::readyRead, this, [this]() {
        if (!m_reply || !m_outFile) return;
        m_outFile->write(m_reply->readAll());
    });

    connect(m_reply, &QNetworkReply::finished, this, [this, savePath]() {
        if (!m_reply) return;

        if (m_outFile) {
            m_outFile->flush();
            m_outFile->close();
        }

        if (m_reply->error() != QNetworkReply::NoError) {
            if (m_progress) m_progress->cancel();

            const QString err = m_reply->errorString();

            // 실패하면 파일 삭제
            if (m_outFile) {
                QFile::remove(savePath);
            }

            QMessageBox::warning(this, "Download", "다운로드 실패: " + err);
        } else {
            if (m_progress) {
                m_progress->setValue(100);
                m_progress->close();
            }
            QMessageBox::information(this, "Download", "저장 완료:\n" + savePath);
        }

        if (m_reply) {
            m_reply->deleteLater();
            m_reply = nullptr;
        }
        if (m_outFile) {
            m_outFile->deleteLater();
            m_outFile = nullptr;
        }
    });
}



void VideoClipWindow::onDownloadProgress(qint64 received, qint64 total)
{
    if (!m_progress) return;

    if (total <= 0) {
        // total 모를 때
        m_progress->setLabelText(QString("Downloading... %1 KB").arg(received / 1024));
        return;
    }

    int percent = static_cast<int>((received * 100) / total);
    m_progress->setValue(percent);
}

bool VideoClipWindow::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
    if (FramelessHelper::handleNativeEvent(this, m_titleBar, eventType, message, result)) {
        return true;
    }
    return QWidget::nativeEvent(eventType, message, result);
}

void VideoClipWindow::changeEvent(QEvent* event)
{
    if (event && event->type() == QEvent::WindowStateChange) {
        if (!isMaximized() && isVisible()) {
            m_normalGeometry = geometry();
        }
        if (m_titleBar) m_titleBar->setMaximized(isMaximized());
        syncWindowLayout();
    }
    QWidget::changeEvent(event);
}
