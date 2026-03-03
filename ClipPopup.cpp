// ClipPopup.cpp
#include "ClipPopup.h"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QFileDialog>
#include <QMessageBox>

#include <QMediaPlayer>
#include <QAudioOutput>
#include <QVideoWidget>

#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QProgressDialog>
#include <QFile>

ClipPopup::ClipPopup(QWidget* parent) : QWidget(parent)
{
    setWindowTitle("Risk Clip");
    setWindowFlags(Qt::Tool | Qt::WindowStaysOnTopHint);
    resize(640, 360);

    buildUi();

    m_player = new QMediaPlayer(this);
    m_audio  = new QAudioOutput(this);
    m_player->setAudioOutput(m_audio);
    m_player->setVideoOutput(m_video);

    m_net = new QNetworkAccessManager(this);

    connect(m_btnDownload, &QPushButton::clicked, this, &ClipPopup::onDownload);
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

    m_btnDownload = new QPushButton("Download", this);
    m_btnClose    = new QPushButton("Close", this);

    auto* btns = new QHBoxLayout();
    btns->addStretch();
    btns->addWidget(m_btnDownload);
    btns->addWidget(m_btnClose);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10,10,10,10);
    root->addLayout(top);
    root->addWidget(m_video, 1);
    root->addLayout(btns);

    setLayout(root);

    // 배경
    setStyleSheet("background-color:#101010;");
}

void ClipPopup::loadAndPlay(const QUrl& url)
{
    m_currentUrl = url;

    const bool canDownload = (url.isValid() && (url.scheme()=="http" || url.scheme()=="https"));
    m_btnDownload->setEnabled(canDownload);

    if (!url.isValid() || url.isEmpty()) return;

    m_player->stop();
    m_player->setSource(url);
    m_player->play();
}

void ClipPopup::playClip(const QString& cam,
                         const QString& eventName,
                         const QString& utcShort,
                         const QUrl& url,
                         int clipSec)
{
    m_labelEvent->setText(QString("Event: %1").arg(eventName));
    m_labelCam->setText(QString("Cam: %1").arg(cam));
    m_labelTime->setText(QString("Time: %1").arg(utcShort));

    loadAndPlay(url);

    // 자동 종료(clipSec + 1초)
    QTimer::singleShot((clipSec + 1) * 1000, this, &ClipPopup::onAutoCloseTimeout);
}

void ClipPopup::onAutoCloseTimeout()
{
    if (m_player) m_player->stop();
    close();
}

void ClipPopup::onDownload()
{
    if (!m_currentUrl.isValid() || m_currentUrl.isEmpty()) {
        QMessageBox::warning(this, "Download", "URL invalid");
        return;
    }
    if (!(m_currentUrl.scheme()=="http" || m_currentUrl.scheme()=="https")) {
        QMessageBox::warning(this, "Download", "HTTP/HTTPS만 다운로드 가능");
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
        QMessageBox::warning(this, "Download", "파일 열기 실패");
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
            QMessageBox::warning(this, "Download", "다운로드 실패: " + m_reply->errorString());
        } else {
            QMessageBox::information(this, "Download", "저장 완료:\n" + savePath);
        }

        if (m_progress) { m_progress->close(); m_progress->deleteLater(); m_progress = nullptr; }
        if (m_reply) { m_reply->deleteLater(); m_reply = nullptr; }
        if (m_outFile) { m_outFile->deleteLater(); m_outFile = nullptr; }
    });
}
