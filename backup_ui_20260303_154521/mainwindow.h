#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTreeWidgetItem>
#include <QListWidgetItem>
#include <QMap>
#include <QVector>
#include <QDateTime>
#include <QUrl>
#include <QHash>
#include <QColor>
#include <QPointF>
#include <QTransform>

#include "MqttSubscriber.h"
#include "MqttEvent.h"
#include "MqttPublisher.h"
#include "MapData.h"

// Forward declarations
class VideoClipWindow;
class AllCctvWindow;
class ClipPopup;
class MiniMapWidget;
class CctvOverlayWidget;
class CaptureCalibOverlay;
class QKeyEvent;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    enum class UserRole {
        Executive,
        ControlRoom
    };

    struct StreamPair {
        QString rawRtsp;
        QString privacyRtsp;
    };

    struct EventItem {
        QString id;
        QString source;        // "mqtt" / "ward" 등
        QString wardName;
        QString message;
        QString privacyClip;
        QString rawClip;
        QDateTime ts;
    };

public:
    explicit MainWindow(const QString& userId = "", QWidget *parent = nullptr);
    ~MainWindow() override;

    void pushEventMessage(const QString& topic, const QString& payload);

protected:
    void resizeEvent(QResizeEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void onWardItemClicked(QTreeWidgetItem* item, int column);
    void onEventClicked(QListWidgetItem* item);
    void onOpenAllCctv();

    void onMqttEvent(const MqttEvent& ev);
    void onMqttLogLine(const QString& s);
    void onHomographyReceived(const QTransform& hImgToWorld);
    void onOverlayCalibPressed(const QPointF& p);
    void onOverlayCalibMoved(const QPointF& p);
    void onOverlayCalibReleased(const QPointF& p);
    void onMiniMapNodeMoved(const QString& id, const QPointF& worldPos);
    void onMiniMapWorldClicked(const QPointF& worldPos);
    void onCentralGoalClicked(const QPointF& widgetPos);
    void onStartCaptureCalib();

private:
    // role
    UserRole resolveRoleFromUserId(const QString& userId) const;
    bool canViewWardRaw() const;

    // init
    void initUiPlaceholders();
    void initWardTree();          // Restricted/Tank/Recon 트리
    void initStreamMap();         // Main1/T1/P1만 실제 등록
    void initCentralStreamMap();  // 기존 중앙 채널 버튼용

    // central live (기존 채널 버튼 방식)
    void switchCentralChannel(int ch);

    // tree 클릭 라우팅용
    void switchCentralStreamTo(const QString& key); // Main1 -> labelCentralView
    void switchTankStreamTo(const QString& key);    // T1    -> labelTankView
    void switchPatrolStreamTo(const QString& key);  // P1    -> labelPatrolView

    // utils
    void appendLog(const QString& line);
    QString clipTitle(const EventItem& ev) const;

    // event style
    void initEventStyleMap();
    bool isDangerEventTopic(const QString& topic) const;
    void initLocalMapFallback();
    void applyOverlayTransform();
    void saveOverlayTweak() const;
    void loadOverlayTweak();
    int findNearestCalibPoint(const QPointF& p, double thPx = 28.0) const;
    void rebuildHomographyFromCalib();
    void syncCentralOverlayGeometry();
    void updateCentralLayerOrder();
    void applyCalibImagePoints(const QVector<QPointF>& imgPts);

private:
    Ui::MainWindow* ui = nullptr;

    // MQTT
    MqttSubscriber* m_mqtt = nullptr;
    MqttPublisher*  m_pub  = nullptr;

    quint64 m_mqttSeq = 0;

    // user/role
    QString  m_userId;
    UserRole m_role;

    // streams
    QMap<int, QString> m_centralRtsp;
    int m_centralCh = 1;

    // tree key -> stream (실제 있는 것만)
    QMap<QString, StreamPair> m_wardStreams;

    // url cache (view별)
    QString m_centralPlayingUrl;
    QString m_tankPlayingUrl;
    QString m_patrolPlayingUrl;

    // events
    QVector<EventItem> m_events;

    // clip windows
    VideoClipWindow* m_clipWin = nullptr;
    AllCctvWindow*   m_allCctvWin = nullptr;
    ClipPopup*       m_clipPopup = nullptr;

    // Map UI
    //MiniMapWidget*      m_mapWidget = nullptr;
    CctvOverlayWidget*  m_overlay   = nullptr;

    // map cache: nodeId -> worldPos
    QMap<QString, QPointF> m_nodePos;

    // target state
    bool    m_hasTarget = false;
    QPointF m_targetWorld;

    // colors
    QHash<QString, QColor> m_eventColor;

    // homography + user tweak
    QTransform m_hWorldToImg;
    bool m_hasHomography = false;
    bool m_homographyLocked = false;
    bool m_mapOverlayEnabled = false;
    bool m_cameraGoalMode = false;
    bool m_patrolPointsEnabled = false;
    double m_tweakDx = 0.0;
    double m_tweakDy = 0.0;
    double m_tweakScale = 1.0;
    double m_tweakRotDeg = 0.0;

    QVector<QPointF> m_calibWorldPts;
    QVector<QPointF> m_calibImgPts;
    int m_dragCalibIdx = -1;
    CaptureCalibOverlay* m_captureOverlay = nullptr;
};

#endif // MAINWINDOW_H
