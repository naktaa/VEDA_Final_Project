#ifndef CLIPPOPUP_H
#define CLIPPOPUP_H
// ClipPopup.h
#pragma once
#include <QWidget>
#include <QKeyEvent>
#include <QRect>
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
class MqttPublisher;
class QSlider;
class TitleBarWidget;
class QEvent;

class ClipPopup : public QWidget {
    Q_OBJECT
public:
    explicit ClipPopup(const QString& brokerHost,
                       int brokerPort,
                       const QString& clientId,
                       QWidget* parent=nullptr);
    ~ClipPopup();

    void playClip(const QString& cam,
                  const QString& eventName,
                  const QString& utcShort,
                  const QUrl& url,
                  int clipSec);

public slots:
    void onDownload();
    void onSend();

private slots:
    void onAutoCloseTimeout();
    void onPlayPause();
    void onStop();
    void onPositionChanged(qint64 pos);
    void onDurationChanged(qint64 dur);
    void onSliderPressed();
    void onSliderReleased();
    void onSliderMoved(int value);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    bool nativeEvent(const QByteArray& eventType, void* message, qintptr* result) override;
    void changeEvent(QEvent* event) override;

private:
    void buildUi();
    void loadAndPlay(const QUrl& url);
    static QString msToTime(qint64 ms);
    void syncWindowLayout();

private:
    TitleBarWidget* m_titleBar = nullptr;
    QLabel* m_labelEvent = nullptr;
    QLabel* m_labelCam = nullptr;
    QLabel* m_labelTime = nullptr;

    QVideoWidget* m_video = nullptr;
    QPushButton* m_btnPlayPause = nullptr;
    QPushButton* m_btnDownload = nullptr;
    QPushButton* m_btnSend = nullptr;
    QPushButton* m_btnClose = nullptr;
    QSlider* m_slider = nullptr;
    QLabel* m_timeLabel = nullptr;

    QMediaPlayer* m_player = nullptr;
    QAudioOutput* m_audio = nullptr;
    bool m_sliderDragging = false;
    qint64 m_durationMs = 0;

    QNetworkAccessManager* m_net = nullptr;
    QNetworkReply* m_reply = nullptr;
    QProgressDialog* m_progress = nullptr;
    QFile* m_outFile = nullptr;
    MqttPublisher* m_pub = nullptr;

    QUrl m_currentUrl;
    QString m_currentCam;
    QString m_currentEventName;
    QString m_currentUtcShort;
    QRect m_normalGeometry;
};


#endif // CLIPPOPUP_H
