// ClipPopup.cpp
#include "ClipPopup.h"
#include "MqttPublisher.h"
#include "TitleBarWidget.h"
#include "FramelessHelper.h"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>

#include <QMediaPlayer>
#include <QAudioOutput>
#include <QVideoWidget>
#include <QSlider>

#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QProgressDialog>
#include <QFile>
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

ClipPopup::ClipPopup(const QString& brokerHost,
                     int brokerPort,
                     const QString& clientId,
                     QWidget* parent) : QWidget(parent)
{
    setWindowTitle("Risk Clip");
    setWindowFlags(Qt::Tool | Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
    resize(640, 360);

    buildUi();

    m_player = new QMediaPlayer(this);
    m_audio  = new QAudioOutput(this);
    m_player->setAudioOutput(m_audio);
    m_player->setVideoOutput(m_video);
    connect(m_player, &QMediaPlayer::positionChanged, this, &ClipPopup::onPositionChanged);
    connect(m_player, &QMediaPlayer::durationChanged, this, &ClipPopup::onDurationChanged);

    m_net = new QNetworkAccessManager(this);
    m_pub = new MqttPublisher(this);
    m_pub->start(brokerHost, brokerPort, clientId);

    connect(m_btnDownload, &QPushButton::clicked, this, &ClipPopup::onDownload);
    connect(m_btnSend, &QPushButton::clicked, this, &ClipPopup::onSend);
    connect(m_btnPlayPause, &QPushButton::clicked, this, &ClipPopup::onPlayPause);
    connect(m_slider, &QSlider::sliderPressed, this, &ClipPopup::onSliderPressed);
    connect(m_slider, &QSlider::sliderReleased, this, &ClipPopup::onSliderReleased);
    connect(m_slider, &QSlider::sliderMoved, this, &ClipPopup::onSliderMoved);
    connect(m_btnClose, &QPushButton::clicked, this, [this]{
        if (m_player) m_player->stop();
        close();
    });
}

ClipPopup::~ClipPopup()
{
    if (m_reply) { m_reply->abort(); m_reply->deleteLater(); }
    if (m_outFile) { m_outFile->close(); m_outFile->deleteLater(); }
}

void ClipPopup::buildUi()
{
    auto* top = new QVBoxLayout();
    m_titleBar = new TitleBarWidget(this);
    m_titleBar->setTitleText("Risk Clip");
    top->addWidget(m_titleBar);

    m_labelEvent = new QLabel("Event:", this);
    m_labelCam   = new QLabel("Cam:", this);
    m_labelTime  = new QLabel("Time:", this);

    m_labelEvent->setStyleSheet("color:white; font-weight:600;");
    m_labelCam->setStyleSheet("color:white;");
    m_labelTime->setStyleSheet("color:white;");

    top->addWidget(m_labelEvent);
    top->addWidget(m_labelCam);
    top->addWidget(m_labelTime);

    m_video = new QVideoWidget(this);
    m_video->setMinimumHeight(220);

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
    m_btnDownload = new QPushButton("Download", this);
    m_btnSend     = new QPushButton("Send", this);
    m_btnClose    = new QPushButton("Close", this);
    m_slider      = new QSlider(Qt::Horizontal, this);
    m_slider->setRange(0, 0);
    m_timeLabel   = new QLabel("00:00 / 00:00", this);

    auto* btns = new QHBoxLayout();
    btns->addWidget(m_btnPlayPause);
    btns->addWidget(m_slider, 1);
    btns->addWidget(m_timeLabel);
    btns->addSpacing(8);
    btns->addWidget(m_btnDownload);
    btns->addWidget(m_btnSend);
    btns->addWidget(m_btnClose);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10,10,10,10);
    root->addLayout(top);
    root->addWidget(m_video, 1);
    root->addLayout(btns);

    setLayout(root);

    // background aligned with main window theme
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

void ClipPopup::syncWindowLayout()
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

void ClipPopup::loadAndPlay(const QUrl& url)
{
    m_currentUrl = url;
    m_btnSend->setEnabled(url.isValid() && !url.isEmpty());

    const bool canDownload = (url.isValid() && (url.scheme()=="http" || url.scheme()=="https"));
    m_btnDownload->setEnabled(canDownload);

    if (!url.isValid() || url.isEmpty()) return;

    m_player->stop();
    m_player->setSource(url);
    m_player->play();
    applyPlaybackButtonState(m_btnPlayPause, true);
}

QString ClipPopup::msToTime(qint64 ms)
{
    const qint64 totalSec = ms / 1000;
    const qint64 min = totalSec / 60;
    const qint64 sec = totalSec % 60;
    return QString("%1:%2")
        .arg(min, 2, 10, QChar('0'))
        .arg(sec, 2, 10, QChar('0'));
}

void ClipPopup::playClip(const QString& cam,
                         const QString& eventName,
                         const QString& utcShort,
                         const QUrl& url,
                         int clipSec)
{
    Q_UNUSED(clipSec);
    m_currentCam = cam;
    m_currentEventName = eventName;
    m_currentUtcShort = utcShort;
    setWindowTitle(QString("Risk Clip - %1").arg(eventName));
    if (m_titleBar) m_titleBar->setTitleText(windowTitle());
    m_labelEvent->setText(QString("Event: %1").arg(eventName));
    m_labelCam->setText(QString("Cam: %1").arg(cam));
    m_labelTime->setText(QString("Time: %1").arg(utcShort));

    loadAndPlay(url);
}

void ClipPopup::onAutoCloseTimeout()
{
}

void ClipPopup::onPlayPause()
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

void ClipPopup::onStop()
{
    if (!m_player) return;
    m_player->stop();
    applyPlaybackButtonState(m_btnPlayPause, false);
}

void ClipPopup::onPositionChanged(qint64 pos)
{
    if (!m_sliderDragging) m_slider->setValue(static_cast<int>(pos));
    if (m_timeLabel) {
        m_timeLabel->setText(QString("%1 / %2").arg(msToTime(pos), msToTime(m_durationMs)));
    }
}

void ClipPopup::onDurationChanged(qint64 dur)
{
    m_durationMs = dur;
    if (m_slider) m_slider->setRange(0, static_cast<int>(dur));
    if (m_timeLabel) {
        m_timeLabel->setText(QString("00:00 / %1").arg(msToTime(dur)));
    }
}

void ClipPopup::onSliderPressed()
{
    m_sliderDragging = true;
}

void ClipPopup::onSliderReleased()
{
    m_sliderDragging = false;
    if (m_player && m_slider) m_player->setPosition(m_slider->value());
}

void ClipPopup::onSliderMoved(int value)
{
    if (m_sliderDragging && m_timeLabel) {
        m_timeLabel->setText(QString("%1 / %2").arg(msToTime(value), msToTime(m_durationMs)));
    }
}

void ClipPopup::onDownload()
{
    if (!m_currentUrl.isValid() || m_currentUrl.isEmpty()) {
        QMessageBox::warning(this, "Download", "URL invalid");
        return;
    }
    if (!(m_currentUrl.scheme()=="http" || m_currentUrl.scheme()=="https")) {
        QMessageBox::warning(this, "Download", "Only HTTP/HTTPS downloads are supported.");
        return;
    }

    const QString savePath = QFileDialog::getSaveFileName(
        this,
        "Save clip",
        "risk_clip.mp4",
        "MP4 Video (*.mp4);;All Files (*.*)"
        );
    if (savePath.isEmpty()) return;

    // cleanup old
    if (m_reply) { m_reply->abort(); m_reply->deleteLater(); m_reply = nullptr; }
    if (m_outFile) { m_outFile->close(); m_outFile->deleteLater(); m_outFile = nullptr; }
    if (m_progress) { m_progress->deleteLater(); m_progress = nullptr; }

    m_outFile = new QFile(savePath, this);
    if (!m_outFile->open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "Download", "Failed to open output file.");
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

    m_progress = new QProgressDialog("Downloading...", "Cancel", 0, 100, this);
    m_progress->setWindowModality(Qt::WindowModal);
    m_progress->setMinimumDuration(0);

    connect(m_progress, &QProgressDialog::canceled, this, [this]() {
        if (m_reply) m_reply->abort();
    });

    connect(m_reply, &QNetworkReply::downloadProgress, this,
            [this](qint64 received, qint64 total){
                if (!m_progress) return;
                if (total <= 0) return;
                m_progress->setValue(int((received * 100) / total));
            });

    connect(m_reply, &QNetworkReply::readyRead, this, [this](){
        if (!m_reply || !m_outFile) return;
        m_outFile->write(m_reply->readAll());
    });

    connect(m_reply, &QNetworkReply::finished, this, [this, savePath](){
        if (m_outFile) { m_outFile->flush(); m_outFile->close(); }

        if (m_reply->error() != QNetworkReply::NoError) {
            QFile::remove(savePath);
            QMessageBox::warning(this, "Download", "Download failed: " + m_reply->errorString());
        } else {
            QMessageBox::information(this, "Download", "Saved to:\n" + savePath);
        }

        if (m_progress) { m_progress->close(); m_progress->deleteLater(); m_progress = nullptr; }
        if (m_reply) { m_reply->deleteLater(); m_reply = nullptr; }
        if (m_outFile) { m_outFile->deleteLater(); m_outFile = nullptr; }
    });
}

void ClipPopup::onSend()
{
    if (!m_pub) {
        QMessageBox::warning(this, "Send", "MQTT publisher is not available.");
        return;
    }
    if (!m_currentUrl.isValid() || m_currentUrl.isEmpty()) {
        QMessageBox::warning(this, "Send", "Clip URL is not available.");
        return;
    }

    QJsonObject payload;
    payload["type"] = "clip_forward_request";
    payload["clip_url"] = m_currentUrl.toString();
    payload["cam"] = m_currentCam;
    payload["event"] = m_currentEventName;
    payload["utc"] = m_currentUtcShort;
    payload["target"] = "control_room";
    payload["requested_by"] = "clip_popup";

    const bool ok = m_pub->publishJson(
        "wiserisk/clips/forward",
        QJsonDocument(payload).toJson(QJsonDocument::Compact),
        1,
        false);

    if (ok) {
        QMessageBox::information(this, "Send", "Forward request sent.");
    } else {
        QMessageBox::warning(this, "Send", "Failed to publish forward request.");
    }
}

void ClipPopup::keyPressEvent(QKeyEvent* event)
{
    if (event && event->key() == Qt::Key_Escape) {
        if (m_player) m_player->stop();
        close();
        event->accept();
        return;
    }
    QWidget::keyPressEvent(event);
}

bool ClipPopup::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
    if (FramelessHelper::handleNativeEvent(this, m_titleBar, eventType, message, result)) {
        return true;
    }
    return QWidget::nativeEvent(eventType, message, result);
}

void ClipPopup::changeEvent(QEvent* event)
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

