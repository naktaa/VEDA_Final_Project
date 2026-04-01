#ifndef VIDEO_CLIP_WINDOW_H
#define VIDEO_CLIP_WINDOW_H

#include <QWidget>
#include <QRect>
#include <QUrl>
#include <QFile>
class QLabel;
class QPushButton;
class QSlider;
class QVideoWidget;

class QMediaPlayer;
class QAudioOutput;
class QEvent;
class TitleBarWidget;

class QNetworkAccessManager;
class QNetworkReply;
class QProgressDialog;

class VideoClipWindow : public QWidget
{
    Q_OBJECT

public:
    enum class UserRole {
        Executive,     // 총책임자: raw 허용
        ControlRoom    // 관제실: privacy만
    };

    explicit VideoClipWindow(QWidget* parent = nullptr);
    ~VideoClipWindow();

    // 이벤트 클릭 시 MainWindow에서 호출
    // - central은 raw/privacy 개념 없으니 rawUrl만 넣어도 됨(privacyUrl 빈 값 가능)
    void openClip(const QString& title,
                  UserRole role,
                  const QUrl& privacyUrl,
                  const QUrl& rawUrl);

private slots:
    void onPlayPause();
    void onStop();
    void onDownload();
    void onClose();

    void onPositionChanged(qint64 pos);
    void onDurationChanged(qint64 dur);
    void onSliderPressed();
    void onSliderReleased();
    void onSliderMoved(int value);

    void onDownloadProgress(qint64 received, qint64 total);


private:
    void buildUi();
    void loadAndPlay(const QUrl& url);
    void syncWindowLayout();

    QUrl chooseUrlByRole(UserRole role, const QUrl& privacyUrl, const QUrl& rawUrl) const;

    static QString msToTime(qint64 ms);
    bool nativeEvent(const QByteArray& eventType, void* message, qintptr* result) override;
    void changeEvent(QEvent* event) override;

private:
    // ui
    TitleBarWidget* m_titleBar = nullptr;
    QLabel* m_titleLabel = nullptr;

    QVideoWidget* m_video = nullptr;

    QPushButton* m_btnPlayPause = nullptr;
    QSlider* m_slider = nullptr;
    QLabel* m_timeLabel = nullptr;

    QPushButton* m_btnDownload = nullptr;
    QPushButton* m_btnClose = nullptr;

    // player
    QMediaPlayer* m_player = nullptr;
    QAudioOutput* m_audio = nullptr;

    bool m_sliderDragging = false;
    qint64 m_durationMs = 0;

    // current clip info
    QString m_title;
    UserRole m_role = UserRole::ControlRoom;
    QUrl m_currentUrl;
    QUrl m_privacyUrl;
    QUrl m_rawUrl;

    // download
    QNetworkAccessManager* m_net = nullptr;
    QNetworkReply* m_reply = nullptr;
    QProgressDialog* m_progress = nullptr;

    QFile* m_outFile = nullptr;
    QRect m_normalGeometry;

};

#endif // VIDEO_CLIP_WINDOW_H
