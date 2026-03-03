#ifndef CLIPPOPUP_H
#define CLIPPOPUP_H
// ClipPopup.h
#pragma once
#include <QWidget>
#include <QUrl>

class QLabel;
class QPushButton;
class QMediaPlayer;
class QAudioOutput;
class QVideoWidget;
class QNetworkAccessManager;
class QNetworkReply;
class QProgressDialog;
class QFile;

class ClipPopup : public QWidget {
    Q_OBJECT
public:
    explicit ClipPopup(QWidget* parent=nullptr);
    ~ClipPopup();

    void playClip(const QString& cam,
                  const QString& eventName,
                  const QString& utcShort,
                  const QUrl& url,
                  int clipSec);

public slots:
    void onDownload();

private slots:
    void onAutoCloseTimeout();

private:
    void buildUi();
    void loadAndPlay(const QUrl& url);

private:
    QLabel* m_labelEvent = nullptr;
    QLabel* m_labelCam = nullptr;
    QLabel* m_labelTime = nullptr;

    QVideoWidget* m_video = nullptr;
    QPushButton* m_btnDownload = nullptr;
    QPushButton* m_btnClose = nullptr;

    QMediaPlayer* m_player = nullptr;
    QAudioOutput* m_audio = nullptr;

    QNetworkAccessManager* m_net = nullptr;
    QNetworkReply* m_reply = nullptr;
    QProgressDialog* m_progress = nullptr;
    QFile* m_outFile = nullptr;

    QUrl m_currentUrl;
};


#endif // CLIPPOPUP_H
