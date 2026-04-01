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
#include <QSet>
#include <QPointF>
#include <QRectF>
#include <QTransform>
#include <QSize>

#include "MqttSubscriber.h"
#include "MqttEvent.h"
#include "MqttPublisher.h"
#include "MapData.h"
#include "PoseOverlayWidget.h"
#include "HumanBoxOverlayWidget.h"
#include "runtime_config.hpp"

// Forward declarations
class VideoClipWindow;
class AllCctvWindow;
class ClipPopup;
class MiniMapWidget;
class CctvOverlayWidget;
class CaptureCalibOverlay;
class SystemUsageWindow;
class RobotStatusWindow;
class QEvent;
class QKeyEvent;
class QShowEvent;
class TitleBarWidget;
class QLabel;
class QComboBox;
class QLineEdit;
class QPushButton;
class QListWidget;
class QListWidgetItem;
class QProcess;
class QSplitter;
class QStackedWidget;
class QTimer;

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

    enum class PrimaryViewMode {
        Map,
        Detection,
        RuView,
        Tank
    };

    struct StreamPair {
        QString rawRtsp;
        QString privacyRtsp;
    };

    struct EventItem {
        QString id;
        QString source;        // "mqtt" / "ward" ??
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
    bool eventFilter(QObject* watched, QEvent* event) override;
    void resizeEvent(QResizeEvent *event) override;
    void showEvent(QShowEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    bool nativeEvent(const QByteArray& eventType, void* message, qintptr* result) override;
    void changeEvent(QEvent* event) override;

private slots:
    void onWardItemClicked(QTreeWidgetItem* item, int column);
    void onEventClicked(QListWidgetItem* item);
    void onEventDoubleClicked(QListWidgetItem* item);
    void onOpenAllCctv();

    void onMqttEvent(const MqttEvent& ev);
    void onMqttLogLine(const QString& s);
    void onHomographyReceived(const QTransform& hImgToWorld);
    void onMapReceived(const MapData& map);
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
    void setupCustomTitleBar();
    void initUiPlaceholders();
    void buildResponsiveLayout();
    void initWardTree();          // Restricted/Tank/Recon ?�리
    void initStreamMap();         // Main1/T1/P1�??�제 ?�록
    void initCentralStreamMap();  // 기존 중앙 채널 버튼??

    // central live (기존 채널 버튼 방식)
    void switchCentralChannel(int ch);

    // tree ?�릭 ?�우?�용
    void switchCentralStreamTo(const QString& key);
    void switchTankStreamTo(const QString& key);
    void switchPatrolStreamTo(const QString& key);

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
    void publishHomographyImageToWorld();
    void syncCentralOverlayGeometry();
    void updateCentralLayerOrder();
    void applyCalibImagePoints(const QVector<QPointF>& imgPts);
    void updateStatusBarText();
    bool isRuViewEvent(const MqttEvent& ev) const;
    void applyRuViewZoneVisual(bool present, double confidence);
    bool publishRuViewControl(bool enabled);
    bool publishTankControl(const QString& group, const QString& command, bool active);
    void bindTankControlButton(QPushButton* button, const QString& group, const QString& command);
    void syncRuViewToggle(bool enabled);
    void setupRuViewInfoUi();
    void setupEventFilterUi();
    void refreshEventFilter();
    bool shouldShowEventItem(const QListWidgetItem* item) const;
    void updateEventCountBadge();
    void loadUiSettings();
    void saveUiSettings() const;
    QString buildEventDedupKey(const MqttEvent& ev, const QString& srcLabel, const QString& locationLabel) const;
    QString formatEventItemText(const QString& firstLine, const QString& secondLine, int duplicateCount) const;
    bool tryMergeDuplicateEvent(const QString& dedupKey,
                                const QString& firstLine,
                                const QString& secondLine,
                                const MqttEvent& ev,
                                const QString& src,
                                const QString& utcShort,
                                const QColor& color);
    void clearRuViewUiState();
    void updateCentralAspectRatio();
    void updateSideVideoAspectRatios();
    void applyMainSplitterDefaultSizes();
    QTransform currentOverlayHomography() const;
    void updatePoseOverlayGeometry();
    void loadRuViewZoneConfig();
    void maybeRaiseRuViewZoneAlert(const MqttEvent& ev);
    void applyPrimaryViewMode(PrimaryViewMode mode);
    void updatePrimaryModeControls();
    void rebuildPrimaryNavList();
    void onPrimaryNavItemClicked(QListWidgetItem* item);
    bool tryOpenMqttDangerClip(QListWidgetItem* item);
    void updateSystemUsageTree();
    void requestGpuUsageUpdate();
    void updateHeaderSummary();
    void refreshEventItemWidget(QListWidgetItem* item);
    QWidget* buildEventItemWidget(QListWidgetItem* item) const;

private:
    Ui::MainWindow* ui = nullptr;
    TitleBarWidget* m_titleBar = nullptr;

    // MQTT
    MqttSubscriber* m_mqtt = nullptr;
    MqttPublisher*  m_pub  = nullptr;
    QtRuntimeConfig m_runtimeConfig;

    quint64 m_mqttSeq = 0;

    // user/role
    QString  m_userId;
    UserRole m_role;
    QString m_statusBaseText;
    QString m_overlayCalibPath;

    // streams
    QMap<int, QString> m_centralRtsp;
    int m_centralCh = 1;

    // tree key -> stream (?�제 ?�는 것만)
    QMap<QString, StreamPair> m_wardStreams;

    // url cache (view�?
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
    PoseOverlayWidget* m_poseOverlay = nullptr;
    HumanBoxOverlayWidget* m_humanBoxOverlay = nullptr;

    // map cache: nodeId -> worldPos
    QMap<QString, QPointF> m_nodePos;

    // target state
    bool    m_hasTarget = false;
    QPointF m_targetWorld;

    // colors
    QHash<QString, QColor> m_eventColor;

    // RuView UI state
    bool m_ruviewOnline = false;
    bool m_ruviewUiEnabled = false;
    bool m_humanBoxEnabled = false;
    QSize m_humanBoxSourceSize;
    QHash<QString, QRect> m_humanBoxes;
    QHash<QString, QRectF> m_humanBoxSmoothed;
    QHash<QString, QDateTime> m_humanBoxSeenUtc;
    QDateTime m_lastHumanBoxRenderUtc;
    QDateTime m_lastHumanEventListUtc;
    bool m_ruviewPresent = false;
    double m_ruviewConfidence = 0.0;
    int m_ruviewActiveNodes = 0;
    int m_ruviewDetectedNodes = 0;
    QDateTime m_ruviewLastSeenUtc;
    QLabel* m_ruviewStatusLabel = nullptr;
    QLabel* m_ruviewStateLabel = nullptr;
    QLabel* m_ruviewConfLabel = nullptr;
    QLabel* m_ruviewNodesLabel = nullptr;
    QLabel* m_headerModeLabel = nullptr;
    QLabel* m_headerUserLabel = nullptr;
    QLabel* m_headerRcStatusLabel = nullptr;
    QLabel* m_headerRuviewStatusLabel = nullptr;
    QComboBox* m_eventFilter = nullptr;
    QLineEdit* m_eventSearchEdit = nullptr;
    QLabel* m_eventCountLabel = nullptr;
    QPushButton* m_eventPauseButton = nullptr;
    QPushButton* m_eventClearButton = nullptr;
    QPushButton* m_autoClipPopupButton = nullptr;
    QPushButton* m_eventSortButton = nullptr;
    bool m_eventListPaused = false;
    bool m_autoClipPopupEnabled = true;
    bool m_eventAutoSortEnabled = true;
    QHash<QString, QString> m_ruviewNodeZoneMap;
    QHash<QString, QDateTime> m_recentCctvZoneSeenUtc;
    QHash<QString, QDateTime> m_recentRuviewAlertUtc;
    QSet<QString> m_ruviewAlertZones;
    QWidget* m_systemUsageCard = nullptr;
    QFrame* m_robotStatusCard = nullptr;
    QLabel* m_systemCpuValueLabel = nullptr;
    QLabel* m_systemMemoryValueLabel = nullptr;
    QLabel* m_systemGpuValueLabel = nullptr;
    QLabel* m_robotConnLabel = nullptr;
    QLabel* m_robotModeLabel = nullptr;
    QLabel* m_robotMissionLabel = nullptr;
    QLabel* m_robotPoseLabel = nullptr;
    QLabel* m_robotSpeedLabel = nullptr;
    QLabel* m_robotCpuValueLabel = nullptr;
    QLabel* m_robotLastSeenLabel = nullptr;
    RobotStatusWindow* m_robotStatusWindow = nullptr;
    MqttEvent m_lastRobotStatusEvent;

    // homography + user tweak
    QTransform m_hWorldToImg;
    bool m_hasHomography = false;
    bool m_homographyLocked = false;
    bool m_homographyFromMqttImage = false;
    bool m_mapOverlayEnabled = false;
    bool m_nodeOverlayEnabled = false;
    bool m_cameraGoalMode = false;
    bool m_patrolPointsEnabled = false;
    double m_centralAspect = 4.0 / 3.0;
    double m_sideVideoAspect = 1.58;
    QSize m_homographyBaseSize;
    double m_tweakDx = 0.0;
    double m_tweakDy = 0.0;
    double m_tweakScale = 1.0;
    double m_tweakRotDeg = 0.0;

    QVector<QPointF> m_calibWorldPts;
    QVector<QPointF> m_calibImgPts;
    int m_dragCalibIdx = -1;
    CaptureCalibOverlay* m_captureOverlay = nullptr;
    QSplitter* m_mainSplitter = nullptr;
    QWidget* m_tankHeaderHost = nullptr;
    QLabel* m_tankControlStatusLabel = nullptr;
    QString m_activePrimaryNavActionToken;
    QWidget* m_patrolHeaderHost = nullptr;
    QStackedWidget* m_primaryViewStack = nullptr;
    QWidget* m_centralVideoCard = nullptr;
    QWidget* m_tankVideoCard = nullptr;
    QListWidget* m_primaryNavList = nullptr;
    PrimaryViewMode m_primaryViewMode = PrimaryViewMode::Map;
    bool m_mapNavExpanded = true;
    QTreeWidgetItem* m_systemUsageRoot = nullptr;
    QTreeWidgetItem* m_cpuUsageItem = nullptr;
    QTreeWidgetItem* m_memoryUsageItem = nullptr;
    QTreeWidgetItem* m_gpuUsageItem = nullptr;
    QTimer* m_systemUsageTimer = nullptr;
    QProcess* m_gpuUsageProcess = nullptr;
    quint64 m_prevCpuIdle = 0;
    quint64 m_prevCpuKernel = 0;
    quint64 m_prevCpuUser = 0;
    bool m_hasCpuSample = false;
    double m_currentCpuUsage = 0.0;
    double m_currentMemoryUsage = 0.0;
    double m_currentGpuUsage = -1.0;
    QVector<double> m_cpuUsageHistory;
    QVector<double> m_memoryUsageHistory;
    QVector<double> m_gpuUsageHistory;
    SystemUsageWindow* m_systemUsageWindow = nullptr;
};

#endif // MAINWINDOW_H


