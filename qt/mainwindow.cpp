#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ClipPopup.h"
#include "video_clip.h"
#include "AllCctvWindow.h"
#include "HomographyCalib.h"
#include "CctvOverlayWidget.h"
#include "MiniMapWidget.h"
#include "CaptureCalibOverlay.h"
#include "SystemUsageWindow.h"
#include "RobotStatusWindow.h"
#include "TitleBarWidget.h"
#include "FramelessHelper.h"
#include "PoseOverlayWidget.h"
#include "HumanBoxOverlayWidget.h"

#include "GstVideoWidget.h"

#include <QTimer>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QDateTime>
#include <QGraphicsDropShadowEffect>
#include <QGuiApplication>
#include <QLineF>
#include <QKeyEvent>
#include <QListWidget>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QScrollBar>
#include <QSettings>
#include <QSignalBlocker>
#include <QStackedLayout>
#include <QStackedWidget>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>
#include <QFont>
#include <QFile>
#include <QMessageBox>
#include <QMouseEvent>
#include <QProcess>
#include <QRegularExpression>
#include <QScreen>
#include <algorithm>
#include <cmath>
#include <limits>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {
constexpr auto kVisionPoseBridgeCommandTopic = "wiserisk/commands/vision_pose_bridge";
constexpr auto kVisionPoseBridgeStatusType = "vision_pose_bridge_status";
constexpr auto kTankControlTopic = "wiserisk/rc/control";
constexpr auto kRuViewZoneConfigPath = "C:/Users/1-14/real_final/start_zone12_dual.sh";
const QSize kCctvFrameSize(1024, 768);
constexpr qint64 kHumanBoxRenderThrottleMs = 180;
constexpr qint64 kHumanEventListThrottleMs = 1200;
constexpr qint64 kHumanBoxKeepAliveMs = 700;
constexpr int kHumanBoxMaxVisible = 12;
constexpr double kHumanBoxOffsetXRatio = 0.0;
constexpr double kHumanBoxOffsetYRatio = 0.035;
constexpr double kHumanBoxPerspectiveOffsetXRatio = 0.0;
constexpr double kHumanBoxSmoothingAlpha = 0.35;

QString normalizeZoneKey(const QString& value)
{
    return value.trimmed().toLower();
}

bool isRuviewAlertZoneKey(const QString& zone)
{
    const QString key = normalizeZoneKey(zone);
    return key == "zone1" || key == "zone2";
}

QString displayRuviewZoneLabel(const QString& zone)
{
    return zone;
}

QStringList splitNodeList(const QString& value)
{
    QStringList out;
    const QStringList parts = value.split(QRegularExpression("[,\\s]+"), Qt::SkipEmptyParts);
    for (const QString& part : parts) {
        const QString token = normalizeZoneKey(part);
        if (!token.isEmpty()) out << token;
    }
    return out;
}

void addZoneNodes(QHash<QString, QString>& nodeToZone, QSet<QString>& zones,
                  const QString& zoneName, const QStringList& nodes)
{
    const QString zone = normalizeZoneKey(zoneName);
    if (zone.isEmpty()) return;
    zones.insert(zone);
    for (const QString& node : nodes) {
        const QString key = normalizeZoneKey(node);
        if (!key.isEmpty()) nodeToZone.insert(key, zone);
    }
}

bool sizeContains(const QSize& outer, const QSize& inner)
{
    return outer.isValid() &&
           inner.isValid() &&
           outer.width() >= inner.width() &&
           outer.height() >= inner.height();
}

bool shouldAcceptHumanBox(const MqttEvent& ev, const QSize& sourceSize)
{
    if (!ev.bbox.isValid()) return false;

    const int w = ev.bbox.width();
    const int h = ev.bbox.height();
    if (w <= 0 || h <= 0) return false;

    if (ev.confidence > 0.0 && ev.confidence < 0.30) return false;

    const QSize basis = sourceSize.isValid() ? sourceSize : QSize(ev.frameW, ev.frameH);
    const int frameW = std::max(1, basis.width());
    const int frameH = std::max(1, basis.height());
    const double frameArea = static_cast<double>(frameW) * static_cast<double>(frameH);

    const int minWidth = std::max(36, static_cast<int>(std::round(frameW * 0.018)));
    const int minHeight = std::max(54, static_cast<int>(std::round(frameH * 0.035)));
    if (w < minWidth || h < minHeight) return false;

    const double aspect = static_cast<double>(w) / static_cast<double>(h);
    if (aspect < 0.12 || aspect > 1.05) return false;

    const double area = static_cast<double>(w) * static_cast<double>(h);
    const double smallAreaThreshold = frameArea * 0.0018;
    if (area < smallAreaThreshold && aspect > 0.72) return false;

    return true;
}

QRect smoothHumanBox(const QRectF& previous, const QRect& current)
{
    if (!previous.isValid()) return current;

    const QRectF cur = QRectF(current);
    const double a = kHumanBoxSmoothingAlpha;
    const double x = previous.x() + (cur.x() - previous.x()) * a;
    const double y = previous.y() + (cur.y() - previous.y()) * a;
    const double w = previous.width() + (cur.width() - previous.width()) * a;
    const double h = previous.height() + (cur.height() - previous.height()) * a;
    return QRectF(x, y, w, h).toAlignedRect();
}

QSize chooseDetectionSourceSize(const QSize& minBounds,
                                const QSize& explicitSourceSize,
                                const QSize& videoFrameSize,
                                const QSize& currentSourceSize)
{
    if (!minBounds.isValid()) {
        if (explicitSourceSize.isValid()) return explicitSourceSize;
        if (currentSourceSize.isValid()) return currentSourceSize;
        return videoFrameSize;
    }

    if (sizeContains(explicitSourceSize, minBounds)) return explicitSourceSize;
    if (sizeContains(currentSourceSize, minBounds)) return currentSourceSize;
    if (sizeContains(videoFrameSize, minBounds)) return videoFrameSize;

    const QVector<QSize> presets = {
        QSize(1280, 720),
        QSize(1920, 1080),
        QSize(2560, 1440),
        QSize(2688, 1520),
        QSize(3072, 1728),
        QSize(3840, 2160),
        QSize(4096, 2160),
        QSize(1024, 768),
        QSize(1280, 960),
        QSize(1600, 1200),
        QSize(2048, 1536),
        QSize(3072, 2304),
        QSize(3840, 2880)
    };

    const double targetAspect =
        (videoFrameSize.isValid() && videoFrameSize.height() > 0)
            ? static_cast<double>(videoFrameSize.width()) / static_cast<double>(videoFrameSize.height())
            : 0.0;

    QSize best;
    double bestAspectDelta = std::numeric_limits<double>::max();
    qint64 bestArea = std::numeric_limits<qint64>::max();
    for (const QSize& preset : presets) {
        if (!sizeContains(preset, minBounds)) continue;
        const double presetAspect =
            preset.height() > 0
                ? static_cast<double>(preset.width()) / static_cast<double>(preset.height())
                : 0.0;
        const double aspectDelta =
            targetAspect > 0.0 ? std::abs(presetAspect - targetAspect) : 0.0;
        const qint64 area = static_cast<qint64>(preset.width()) * static_cast<qint64>(preset.height());
        if (aspectDelta < bestAspectDelta ||
            (std::abs(aspectDelta - bestAspectDelta) < 0.0001 && area < bestArea)) {
            bestAspectDelta = aspectDelta;
            bestArea = area;
            best = preset;
        }
    }
    if (best.isValid()) return best;

    return QSize(std::max(1, minBounds.width()), std::max(1, minBounds.height()));
}

QVector<QPointF> buildCalibPolyline(const QVector<QPointF>& pts)
{
    if (pts.size() < 4) return {};
    return {
        pts[2],
        pts[3],
        pts[1],
        pts[0],
        pts[2]
    };
}

QVector<QPointF> closedPolylineFromMap(const QVector<QPointF>& polyline)
{
    if (polyline.isEmpty()) return {};
    QVector<QPointF> out = polyline;
    if (out.front() != out.back()) out.push_back(out.front());
    return out;
}

#ifdef Q_OS_WIN
quint64 fileTimeToUInt64(const FILETIME& ft)
{
    ULARGE_INTEGER value{};
    value.LowPart = ft.dwLowDateTime;
    value.HighPart = ft.dwHighDateTime;
    return value.QuadPart;
}
#endif

void appendUsageHistory(QVector<double>& history, double value, int maxCount = 40)
{
    history.push_back(std::clamp(value, 0.0, 100.0));
    while (history.size() > maxCount) history.removeFirst();
}
}

// ---------------- role ----------------
MainWindow::UserRole MainWindow::resolveRoleFromUserId(const QString& userId) const
{
    if (userId == "hospital_director" || userId == "vice_director")
        return UserRole::Executive;
    return UserRole::ControlRoom;
}

bool MainWindow::canViewWardRaw() const
{
    return m_role == UserRole::Executive;
}

// ---------------- ctor/dtor ----------------
MainWindow::MainWindow(const QString& userId, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_userId(userId)
    , m_role(resolveRoleFromUserId(userId))
{
    ui->setupUi(this);
    setupCustomTitleBar();
    buildResponsiveLayout();
    // centralOverlayHost ?占쎌뿉?占쎈쭔 寃뱀튂占?援ъ꽦
    auto* stack = new QStackedLayout(ui->centralOverlayHost);
    stack->setContentsMargins(0,0,0,0);
    stack->setStackingMode(QStackedLayout::StackAll); // ???占쎌떖: ??蹂댁씠占?寃뱀묠

    stack->addWidget(ui->labelCentralView);
    stack->addWidget(ui->labelCentralMap);

    // Keep legacy MiniMap hidden. Calibrated map is rendered on m_overlay.
    ui->labelCentralMap->hide();


    connect(ui->btnMap, &QPushButton::clicked, this, [this]{
        m_mapOverlayEnabled = !m_mapOverlayEnabled;
        if (m_overlay) m_overlay->setMapVisible(m_mapOverlayEnabled);
        if (m_captureOverlay) m_captureOverlay->setMapOverlayVisible(m_mapOverlayEnabled);
        syncCentralOverlayGeometry();
        updateCentralLayerOrder();
        saveUiSettings();
        rebuildPrimaryNavList();
        appendLog(QString("Map overlay %1").arg(m_mapOverlayEnabled ? "ON" : "OFF"));
    });
    if (auto* btnRc = findChild<QPushButton*>("btnRC")) {
        btnRc->setCheckable(true);
        btnRc->setAutoExclusive(false);
        btnRc->setChecked(false);
        connect(btnRc, &QPushButton::toggled, this, [this](bool on){
            m_nodeOverlayEnabled = false;
            m_cameraGoalMode = on;
            m_patrolPointsEnabled = on;
            if (m_overlay) {
                m_overlay->setNodesVisible(false);
                m_overlay->setGoalClickMode(on);
                if (!on) {
                    m_overlay->setGoalMarkerVisible(false);
                }
            }
            if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
                rcMap->setVisible(false);
                rcMap->setGoalMarkerVisible(false);
            }
            syncCentralOverlayGeometry();
            updateCentralLayerOrder();
            rebuildPrimaryNavList();
            appendLog(QString("RC goal mode %1").arg(on ? "ON" : "OFF"));
        });
        btnRc->hide();
    }
    if (auto* ruviewBtn = findChild<QPushButton*>("ruviewbtn")) {
        ruviewBtn->setCheckable(true);
        ruviewBtn->setAutoExclusive(false);
        ruviewBtn->setChecked(false);
        connect(ruviewBtn, &QPushButton::toggled, this, [this](bool on){
            const bool published = publishRuViewControl(on);
            if (!published) {
                syncRuViewToggle(!on);
                appendLog(QString("RuView control publish failed (%1)")
                              .arg(on ? "enable" : "disable"));
                return;
            }

            m_ruviewUiEnabled = on;
            if (!on) {
                clearRuViewUiState();
            }
            updateStatusBarText();
            refreshEventFilter();
            saveUiSettings();
            rebuildPrimaryNavList();
            appendLog(QString("RuView control command sent: %1").arg(on ? "ON" : "OFF"));
        });
    }
    if (auto* humanBtn = findChild<QPushButton*>("Humanbtn")) {
        humanBtn->setCheckable(true);
        humanBtn->setAutoExclusive(false);
        humanBtn->setChecked(false);
        connect(humanBtn, &QPushButton::toggled, this, [this](bool on){
            m_humanBoxEnabled = on;
            if (m_mqtt) m_mqtt->setHumanDetectionEnabled(on);
            if (!on) {
                m_humanBoxSourceSize = QSize();
                m_humanBoxes.clear();
                m_humanBoxSmoothed.clear();
                m_humanBoxSeenUtc.clear();
                if (m_humanBoxOverlay) m_humanBoxOverlay->setHumanBoxes({}, false);
            }
            saveUiSettings();
            rebuildPrimaryNavList();
            appendLog(QString("Detection %1").arg(on ? "ON" : "OFF"));
        });
    }
    if (auto* btnCapture = findChild<QPushButton*>("btnCapture")) {
        connect(btnCapture, &QPushButton::clicked, this, &MainWindow::onStartCaptureCalib);
    } else if (auto* btnCaptureAlt = findChild<QPushButton*>("btnMap_2")) {
        connect(btnCaptureAlt, &QPushButton::clicked, this, &MainWindow::onStartCaptureCalib);
    }
    if (auto* patrolStartBtn = findChild<QPushButton*>("patrolstartbtn")) {
        patrolStartBtn->hide();
    }

    // init
    initUiPlaceholders();
    setupRuViewInfoUi();
    setupEventFilterUi();
    initCentralStreamMap();
    initStreamMap();
    initWardTree();
    initEventStyleMap();
    initLocalMapFallback();
    loadOverlayTweak();
    loadRuViewZoneConfig();
    updateSystemUsageTree();
    m_systemUsageTimer = new QTimer(this);
    m_systemUsageTimer->setInterval(2000);
    connect(m_systemUsageTimer, &QTimer::timeout, this, &MainWindow::updateSystemUsageTree);
    m_systemUsageTimer->start();

    // MQTT
    m_mqtt = new MqttSubscriber(nullptr); // parent 二쇰㈃ moveToThread?占쎌꽌 臾몄젣?占쎌뿀吏
    m_mqtt->setHumanDetectionEnabled(m_humanBoxEnabled);
    connect(m_mqtt, &MqttSubscriber::eventReceived,
            this, &MainWindow::onMqttEvent, Qt::QueuedConnection);
    connect(m_mqtt, &MqttSubscriber::logLine,
            this, &MainWindow::onMqttLogLine, Qt::QueuedConnection);
    connect(m_mqtt, &MqttSubscriber::homographyReceived,
            this, &MainWindow::onHomographyReceived, Qt::QueuedConnection);
    connect(m_mqtt, &MqttSubscriber::mapReceived,
            this, &MainWindow::onMapReceived, Qt::QueuedConnection);
    connect(m_mqtt, &MqttSubscriber::poseReceived, this, [this](const PoseFrame& frame){
        if (!m_poseOverlay) return;
        if (!m_ruviewUiEnabled) {
            m_poseOverlay->setPoseFrame(PoseFrame{});
            return;
        }
        m_ruviewOnline = true;
        m_ruviewPresent = frame.person.detected;
        m_ruviewConfidence = std::max(0.0, std::min(1.0, static_cast<double>(frame.person.score)));
        m_ruviewLastSeenUtc = QDateTime::currentDateTimeUtc();
        applyRuViewZoneVisual(frame.person.detected, m_ruviewConfidence);
        updateStatusBarText();
        m_poseOverlay->setPoseFrame(frame);
        if (ui && ui->labelCentralView) {
            QSize sourceSize(frame.frameW, frame.frameH);
            if (!sourceSize.isValid()) {
                sourceSize = ui->labelCentralView->currentFrameSize();
            }
            m_poseOverlay->setVideoGeometry(
                ui->labelCentralView->videoDisplayRect(),
                sourceSize
            );
        } else {
            updatePoseOverlayGeometry();
        }
        m_poseOverlay->raise();
    }, Qt::QueuedConnection);
    m_pub = new MqttPublisher(this);
    connect(m_pub, &MqttPublisher::logLine,
            this, &MainWindow::onMqttLogLine, Qt::QueuedConnection);

    // Central overlay (寃쎈낫 ?占쎈쾭?占쎌씠)
    if (ui->centralOverlayHost) {
        m_overlay = new CctvOverlayWidget(ui->centralOverlayHost);
        m_overlay->setGeometry(ui->centralOverlayHost->contentsRect());
        m_overlay->setCalibrationMode(false);
        connect(m_overlay, &CctvOverlayWidget::calibrationPressed,
                this, &MainWindow::onOverlayCalibPressed);
        connect(m_overlay, &CctvOverlayWidget::calibrationMoved,
                this, &MainWindow::onOverlayCalibMoved);
        connect(m_overlay, &CctvOverlayWidget::calibrationReleased,
                this, &MainWindow::onOverlayCalibReleased);
        connect(m_overlay, &CctvOverlayWidget::goalClicked,
                this, &MainWindow::onCentralGoalClicked);

        // Initialize overlay map geometry from current calibration world points.
        if (m_calibWorldPts.size() >= 4) {
            QVector<OverlayNode> ons = {
                {"10", m_calibWorldPts[0]},
                {"11", m_calibWorldPts[1]},
                {"12", m_calibWorldPts[2]},
                {"13", m_calibWorldPts[3]}
            };
            QVector<QPointF> poly = buildCalibPolyline(m_calibWorldPts);
            m_overlay->setNodes(ons);
            m_overlay->setNodesVisible(m_nodeOverlayEnabled);
            m_overlay->setPolyline(poly);
        }
        m_overlay->setMapVisible(m_mapOverlayEnabled);
        m_overlay->raise();
        m_overlay->show();
    }

    if (ui->centralOverlayHost) {
        m_poseOverlay = new PoseOverlayWidget(ui->centralOverlayHost);
        m_poseOverlay->setGeometry(ui->centralOverlayHost->contentsRect());
        m_poseOverlay->show();
        m_poseOverlay->raise();
    }
    if (ui->centralOverlayHost) {
        m_humanBoxOverlay = new HumanBoxOverlayWidget(ui->centralOverlayHost);
        m_humanBoxOverlay->setGeometry(ui->centralOverlayHost->contentsRect());
        m_humanBoxOverlay->show();
        m_humanBoxOverlay->raise();
    }

    // Map overlay (UI: labelCentralMap is MiniMapWidget)
    if (ui->labelCentralMap) {
        ui->labelCentralMap->setWorldSize(5.0, 5.0);
        // View-only overlay: all map editing is done in capture calibration mode.
        ui->labelCentralMap->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        connect(ui->labelCentralMap, &MiniMapWidget::nodeMoved,
                this, &MainWindow::onMiniMapNodeMoved);
        connect(ui->labelCentralMap, &MiniMapWidget::worldClicked,
                this, &MainWindow::onMiniMapWorldClicked);
        ui->labelCentralMap->hide();
        // (?占쏀깮) 吏???占쎈┃ 留됯퀬 CCTV ?占쎈┃占?諛쏄쾶
        // ui->labelCentralMap->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    } else {
        appendLog("labelCentralMap is null (Promote/objectName ?占쎌씤)");
    }
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->setWorldSize(5.0, 5.0);
        rcMap->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        rcMap->setDrawMapGeometry(true);
        rcMap->setWorldClickEnabled(false);
        connect(rcMap, &MiniMapWidget::worldClicked,
                this, &MainWindow::onMiniMapWorldClicked);
        rcMap->setGoalMarkerVisible(false); // Use CCTV overlay marker for visual consistency.
        rcMap->hide();
    }
    if (auto* capMap = findChild<MiniMapWidget*>("labelCentralCapture")) {
        capMap->setWorldSize(5.0, 5.0);
        capMap->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        capMap->hide();
    }
    m_mapOverlayEnabled = false;
    m_nodeOverlayEnabled = false;

    // 釉뚮줈占??占쎈쾭 IP占??占쎌냽
    m_mqtt->start("192.168.100.7", 1883, "wiserisk/#");
    m_pub->start("192.168.100.7", 1883, "qt-main-goal-pub");

    // UI connect
    connect(ui->btnOpenAllCctv, &QPushButton::clicked,
            this, &MainWindow::onOpenAllCctv);

    connect(ui->listWardCctv, &QTreeWidget::itemClicked,
            this, &MainWindow::onWardItemClicked);

    connect(ui->listEvent, &QListWidget::itemClicked,
            this, &MainWindow::onEventClicked);
    connect(ui->listEvent, &QListWidget::itemDoubleClicked,
            this, &MainWindow::onEventDoubleClicked);

    // status
    const QString roleText =
        (m_role == UserRole::Executive) ? "Executive" : "ControlRoom";
    m_statusBaseText = QString("Login: %1 | Role: %2")
                           .arg(m_userId.isEmpty() ? "Unknown" : m_userId, roleText);
    updateStatusBarText();
    appendLog("System started");
    loadUiSettings();

    auto* ruviewStatusTimer = new QTimer(this);
    connect(ruviewStatusTimer, &QTimer::timeout, this, [this]{
        if (m_ruviewOnline && m_ruviewLastSeenUtc.isValid()) {
            const qint64 ageMs = m_ruviewLastSeenUtc.msecsTo(QDateTime::currentDateTimeUtc());
            if (ageMs > 5000) {
                m_ruviewOnline = false;
                updateStatusBarText();
            }
        } else {
            updateStatusBarText();
        }
    });
    ruviewStatusTimer->start(1000);

    // ??以묒븰 湲곕낯 梨꾨꼸 (Central CCTV 踰꾪듉/濡쒖쭅占??占쎌씪)
    QTimer::singleShot(0, this, [this]{
        applyMainSplitterDefaultSizes();
        updateCentralAspectRatio();
        updateSideVideoAspectRatios();
        syncCentralOverlayGeometry();
        switchCentralChannel(1); // centralcctv 占?濡쒖쭅 ?占쏙옙?/?占쎌떎???占쏀뻾
        updateCentralLayerOrder();
    });
    QTimer::singleShot(120, this, [this]{
        applyMainSplitterDefaultSizes();
        updateCentralAspectRatio();
        updateSideVideoAspectRatios();
        syncCentralOverlayGeometry();
        updateCentralLayerOrder();
    });
}

MainWindow::~MainWindow()
{
    if (m_mqtt) {
        m_mqtt->stop();
        delete m_mqtt;
        m_mqtt = nullptr;
    }
    if (m_pub) {
        m_pub->stop();
        m_pub = nullptr;
    }

    // GstVideoWidget stop
    if (ui->labelCentralView) ui->labelCentralView->stopStream();
    if (ui->labelTankView)    ui->labelTankView->stopStream();

    delete ui;
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == m_systemUsageCard && event && event->type() == QEvent::MouseButtonRelease) {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
            if (!m_systemUsageWindow) {
                m_systemUsageWindow = new SystemUsageWindow(this);
                m_systemUsageWindow->setAttribute(Qt::WA_DeleteOnClose, false);
            }
            m_systemUsageWindow->setUsageData(
                m_currentCpuUsage,
                m_currentMemoryUsage,
                m_currentGpuUsage,
                m_cpuUsageHistory,
                m_memoryUsageHistory,
                m_gpuUsageHistory);
            m_systemUsageWindow->show();
            m_systemUsageWindow->raise();
            m_systemUsageWindow->activateWindow();
            return true;
        }
    }

    if (watched == m_robotStatusCard && event && event->type() == QEvent::MouseButtonRelease) {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
            if (!m_robotStatusWindow) {
                m_robotStatusWindow = new RobotStatusWindow(this);
            }
            m_robotStatusWindow->setRobotStatus(m_lastRobotStatusEvent);
            m_robotStatusWindow->show();
            m_robotStatusWindow->raise();
            m_robotStatusWindow->activateWindow();
            return true;
        }
    }

    return QMainWindow::eventFilter(watched, event);
}

// ---------------- init ----------------
void MainWindow::initUiPlaceholders()
{
    setStyleSheet(R"(
QMainWindow, QWidget#centralWidget {
    background-color: #171924;
    color: #e7ebff;
}
QWidget#TitleBarWidget {
    background-color: #151924;
}
QLabel#titleText {
    background-color: transparent;
    color: #eaf0ff;
    font-weight: 700;
}
QWidget#platformNameBar {
    background-color: #202636;
    border: 1px solid #353f5d;
    border-radius: 8px;
}
QLabel#platformNameLabel {
    background: transparent;
    color: #f5f7ff;
    font-weight: 800;
    letter-spacing: 0.8px;
}
QLabel[headerMeta="true"] {
    background-color: #2a3247;
    color: #d7e0ff;
    border: 1px solid #3f4c70;
    border-radius: 10px;
    padding: 3px 10px;
    font-size: 10px;
    font-weight: 700;
}
QLabel[headerMeta="true"][accent="status"] {
    background-color: #213d34;
    border-color: #2d7962;
    color: #bff5db;
}
QLabel[headerMeta="true"][accent="danger"] {
    background-color: #41272d;
    border-color: #8a4c57;
    color: #ffd4dc;
}
QWidget#navPanelHeader {
    background-color: #262d40;
    border: 1px solid #353f5c;
    border-radius: 9px;
}
QLabel#navPanelEyebrow {
    color: #8e9bb7;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
}
QLabel#navPanelTitle {
    color: #eef3ff;
    font-size: 18px;
    font-weight: 800;
}
QLabel#navPanelSubtitle {
    color: #99a5bf;
    font-size: 11px;
}
QLabel[navIcon="true"] {
    border-radius: 7px;
    color: #ffffff;
    font-size: 11px;
    font-weight: 800;
    min-width: 28px;
    max-width: 28px;
    min-height: 28px;
    max-height: 28px;
    qproperty-alignment: AlignCenter;
}
QLabel[navIcon="true"][mode="map"] {
    background-color: #5667d8;
}
QLabel[navIcon="true"][mode="detection"] {
    background-color: #6f52d9;
}
QLabel[navIcon="true"][mode="ruview"] {
    background-color: #2b8f72;
}
QLabel[navIcon="true"][mode="tank"] {
    background-color: #b37a2f;
}
QLabel[navIcon="true"][mode="allcctv"] {
    background-color: #2d9ab0;
}
QGroupBox {
    border: 1px solid #303650;
    border-radius: 6px;
    margin-top: 0px;
    padding-top: 0px;
    background-color: #1d2231;
    font-weight: 600;
}
QGroupBox::title {
    margin: 0px;
    padding: 0px;
    color: transparent;
}
QFrame#centralCard {
    background-color: #1d2333;
    border: 1px solid #2f3852;
    border-radius: 10px;
}
QFrame#centralCard[visualPriority="primary"] {
    background-color: #20283b;
    border: 1px solid #4a6293;
}
QFrame#centralCard[visualPriority="secondary"] {
    background-color: #1a2130;
    border: 1px solid #2c3550;
}
QWidget#centralAspectHost,
QWidget#tankAspectHost {
    background-color: transparent;
}
QWidget#centralOverlayHost {
    background-color: #05070d;
    border: 1px solid #32476e;
    border-radius: 12px;
}
QLabel#sideVideoHeader {
    background-color: #232b3f;
    color: #dbe6ff;
    border: 1px solid #34405c;
    border-radius: 6px;
    font-weight: 700;
    padding: 5px 10px;
}
QWidget#sideVideoHeaderHost {
    background-color: transparent;
}
QSplitter#mainSplitter::handle {
    background: #1b2030;
}
QPushButton {
    background-color: #2a334c;
    color: #edf1ff;
    border: 1px solid #435178;
    border-radius: 6px;
    padding: 6px 10px;
    min-height: 20px;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #34405f;
}
QPushButton:pressed {
    background-color: #24304b;
    border-color: #5475b3;
    color: #ffffff;
}
QPushButton[eventControl="secondary"] {
    background-color: #262f45;
    border-color: #3c496b;
}
QPushButton[eventControl="secondary"]:hover {
    background-color: #313d5a;
}
QPushButton[eventControl="secondary"]:pressed,
QPushButton[eventControl="secondary"]:checked {
    background-color: #394a70;
    border-color: #6985bf;
}
QPushButton[eventControl="danger"] {
    background-color: #402c35;
    border-color: #825060;
}
QPushButton[eventControl="danger"]:hover {
    background-color: #513541;
}
QPushButton[eventControl="danger"]:pressed,
QPushButton[eventControl="danger"]:checked {
    background-color: #713f4d;
    border-color: #bd6a7e;
}
QPushButton[eventControl="toggle"] {
    background-color: #213947;
    border-color: #3c6f89;
}
QPushButton[eventControl="toggle"]:hover {
    background-color: #274756;
}
QPushButton[eventControl="toggle"]:checked,
QPushButton[eventControl="toggle"]:pressed {
    background-color: #2d7ca1;
    border-color: #67b8df;
}
QPushButton[eventControl="primary"] {
    background-color: #465ed1;
    border-color: #7087f6;
}
QPushButton[eventControl="primary"]:hover {
    background-color: #5870e3;
}
QPushButton[eventControl="primary"]:pressed,
QPushButton[eventControl="primary"]:checked {
    background-color: #6a5bdb;
    border-color: #958aff;
}
QListWidget, QTreeWidget, QPlainTextEdit {
    background-color: #1b2030;
    color: #dce3ff;
    border: 1px solid #303650;
    border-radius: 6px;
    selection-background-color: #5b48c8;
    selection-color: #ffffff;
}
QListWidget#primaryNavList {
    background-color: #222939;
    border: 1px solid #415071;
    border-radius: 9px;
    padding: 10px 8px;
    outline: none;
}
QListWidget#primaryNavList::item {
    background-color: transparent;
    border: none;
    border-radius: 6px;
    margin: 0 0 8px 0;
    padding: 0px;
    color: #5f6b84;
}
QListWidget#primaryNavList::item:selected {
    background-color: #37435d;
    color: #eef3ff;
}
QListWidget#primaryNavList::item:hover:!selected {
    background-color: #313b53;
    color: #f0f4ff;
}
QFrame[navItemFrame="true"] {
    background-color: #273149;
    border: 1px solid #31405e;
    border-radius: 10px;
}
QFrame[navItemFrame="true"][selected="true"] {
    background-color: #344667;
    border: 1px solid #6882b8;
}
QFrame[navItemFrame="true"][action="true"] {
    background-color: #252d3e;
    border: 1px solid #33415e;
}
QFrame[navItemFrame="true"][action="true"][selected="true"] {
    background-color: #30405d;
    border: 1px solid #5f78ab;
}
QFrame[navGlow="true"] {
    background-color: transparent;
    border: 1px solid transparent;
    border-radius: 10px;
}
QFrame[navGlow="true"][selected="true"] {
    border-color: rgba(130, 155, 255, 0.35);
    background-color: rgba(86, 103, 216, 0.08);
}
QLabel[navTitle="true"] {
    color: #eef3ff;
    font-weight: 700;
    font-size: 14px;
}
QLabel[navTitle="true"][action="true"] {
    color: #c3cee6;
    font-size: 12px;
    font-weight: 600;
}
QLabel[navSubtitle="true"] {
    color: #95a3bf;
    font-size: 11px;
    qproperty-wordWrap: true;
}
QLabel[navBadge="true"] {
    background-color: #dff7ef;
    color: #35b58a;
    border-radius: 5px;
    padding: 1px 7px;
    font-size: 10px;
    font-weight: 700;
}
QLabel[navBadge="true"][active="false"] {
    background-color: #31394f;
    color: #98a3bc;
}
QLabel[navBadge="true"][mode="map"] {
    background-color: #e7e9fb;
    color: #5667d8;
    border: 1px solid #cfd5fb;
    min-width: 14px;
    padding: 0px 5px 0px 4px;
    font-size: 11px;
    font-weight: 800;
}
QLabel[navBadge="true"][mode="detection"] {
    background-color: #ece6fb;
    color: #6f52d9;
}
QLabel[navBadge="true"][mode="ruview"] {
    background-color: #e4f5ef;
    color: #2b8f72;
}
QLabel[navBadge="true"][mode="tank"] {
    background-color: #f9eddc;
    color: #b37a2f;
}
QLabel[navBadge="true"][mode="allcctv"] {
    background-color: #e3f3f7;
    color: #2d9ab0;
}
QLabel[navIndicator="true"] {
    background-color: #6d5efc;
    border-radius: 3px;
}
QLabel[navIndicator="true"][mode="map"] {
    background-color: #5667d8;
}
QLabel[navIndicator="true"][mode="detection"] {
    background-color: #6f52d9;
}
QLabel[navIndicator="true"][mode="ruview"] {
    background-color: #2b8f72;
}
QLabel[navIndicator="true"][mode="tank"] {
    background-color: #b37a2f;
}
QLabel[navIndicator="true"][mode="allcctv"] {
    background-color: #2d9ab0;
}
QTreeWidget {
    font-weight: 700;
}
QLabel {
    background-color: transparent;
    color: #e8edff;
}
QWidget#ruviewInfoCard {
    background-color: #20273a;
    border: 1px solid #364463;
    border-radius: 10px;
}
QGroupBox#groupMsg {
    background-color: #1e2434;
    border: 1px solid #34415f;
    border-radius: 10px;
}
QGroupBox#groupWard {
    background-color: #1d2433;
    border: 1px solid #34415f;
    border-radius: 10px;
}
QFrame#robotStatusCard {
    background-color: #1e2535;
    border: 1px solid #364361;
    border-radius: 10px;
}
QFrame#tankControlCard {
    background-color: #1b2231;
    border: 1px solid #34425f;
    border-radius: 10px;
}
QLabel[tankControlTitle="true"] {
    color: #f2f6ff;
    font-size: 13px;
    font-weight: 800;
}
QLabel[tankControlStatus="true"] {
    color: #95a4c2;
    font-size: 11px;
    font-weight: 700;
}
QPushButton[tankControlButton="true"] {
    min-width: 84px;
    min-height: 34px;
    border-radius: 8px;
    background-color: #27344d;
    border: 1px solid #41537a;
    color: #eef3ff;
    padding: 8px 10px;
}
QPushButton[tankControlButton="true"]:hover {
    background-color: #324362;
}
QPushButton[tankControlButton="true"][tankGroup="drive"]:pressed {
    background-color: #9d6a24;
    border-color: #d59a48;
}
QPushButton[tankControlButton="true"][tankGroup="ptz"]:pressed {
    background-color: #276e93;
    border-color: #59aad0;
}
QLabel[robotTitle="true"] {
    color: #f2f6ff;
    font-size: 15px;
    font-weight: 800;
}
QLabel[robotOpen="true"] {
    color: #8ea1c8;
    font-size: 10px;
    font-weight: 700;
    background-color: #293249;
    border: 1px solid #435478;
    border-radius: 8px;
    padding: 2px 8px;
}
QLabel[robotLabel="true"] {
    color: #91a0bd;
    font-size: 10px;
    font-weight: 700;
    text-transform: uppercase;
}
QLabel[robotValue="true"] {
    color: #eef3ff;
    font-size: 13px;
    font-weight: 800;
}
QLabel[robotConn="online"] {
    color: #50d89d;
}
QLabel[robotConn="offline"] {
    color: #ff8181;
}
QFrame[statusMetricCard="true"] {
    background-color: #293249;
    border: 1px solid #435478;
    border-radius: 10px;
}
QLabel[statusMetricLabel="true"] {
    color: #8ea0c5;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.6px;
}
QLabel[statusMetricValue="true"] {
    color: #f3f7ff;
    font-size: 20px;
    font-weight: 800;
}
QLabel[statusMetricValue="true"][level="warn"] {
    color: #f2c66d;
}
QLabel[statusMetricValue="true"][level="danger"] {
    color: #ff8d8d;
}
QWidget#eventHeaderRow {
    background-color: transparent;
}
QLabel[eventSectionEyebrow="true"] {
    color: #8a98b8;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.8px;
}
QLabel[eventSectionTitle="true"] {
    color: #f3f6ff;
    font-size: 16px;
    font-weight: 800;
}
QLabel#eventCountBadge {
    background-color: #344667;
    color: #eef3ff;
    border: 1px solid #5871a8;
    border-radius: 10px;
    padding: 3px 10px;
    font-size: 11px;
    font-weight: 800;
}
QFrame[eventItemCard="true"] {
    background-color: #252d3f;
    border: 1px solid #33415e;
    border-radius: 10px;
}
QFrame[eventItemCard="true"][severity="danger"] {
    border-color: #9a535d;
    background-color: #2d2429;
}
QFrame[eventItemCard="true"][severity="warn"] {
    border-color: #8f6a41;
    background-color: #2e2922;
}
QFrame[eventAccent="true"] {
    border-radius: 3px;
    min-width: 5px;
    max-width: 5px;
    background-color: #6d7da5;
}
QLabel[eventMeta="true"] {
    color: #90a0c2;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.4px;
}
QLabel[eventTitle="true"] {
    color: #f0f5ff;
    font-size: 12px;
    font-weight: 800;
}
QLabel[eventSubtitle="true"] {
    color: #b8c4de;
    font-size: 11px;
    font-weight: 600;
}
QLabel[eventBadge="true"] {
    border-radius: 8px;
    padding: 2px 8px;
    font-size: 10px;
    font-weight: 800;
    color: #eef4ff;
    background-color: #425273;
}
QLabel[eventBadge="true"][kind="danger"] {
    background-color: #8f4b57;
}
QLabel[eventBadge="true"][kind="warn"] {
    background-color: #8d7040;
}
QLabel[eventBadge="true"][kind="normal"] {
    background-color: #3a557d;
}
QWidget#eventFilterPanel {
    background-color: #1a2130;
    border: 1px solid #303c57;
    border-radius: 10px;
}
QLabel#eventFilterLabel {
    color: #eef3ff;
    font-size: 10px;
    font-weight: 700;
    background-color: #2a3348;
    padding: 4px 10px;
    border: 1px solid #3a4967;
    border-radius: 8px;
}
QToolButton#logToggleButton {
    background-color: #27324a;
    color: #dce5ff;
    border: 1px solid #44557d;
    border-radius: 8px;
    padding: 6px 10px;
    font-weight: 700;
}
QToolButton#logToggleButton:hover {
    background-color: #31405d;
}
QWidget#eventLogContainer {
    background-color: transparent;
}
QLabel[ruCardTitle="true"] {
    color: #eff5ff;
    font-size: 13px;
    font-weight: 800;
}
QLabel[ruCardValue="true"] {
    color: #b2c0de;
    font-size: 11px;
    font-weight: 700;
}
QProgressBar#robotBatteryBar {
    min-height: 10px;
    max-height: 10px;
    border: 1px solid #3a4462;
    border-radius: 5px;
    background-color: #151a28;
}
QProgressBar#robotBatteryBar::chunk {
    border-radius: 4px;
    background-color: #4db1ff;
}
QComboBox#eventFilterCombo {
    background-color: #2c354a;
    color: #eef3ff;
    font-weight: 700;
    border: 1px solid #3c4866;
    border-radius: 8px;
    padding: 4px 10px;
}
QComboBox#eventFilterCombo::drop-down {
    width: 24px;
    border: none;
    border-left: 1px solid #3c4866;
    subcontrol-origin: padding;
    subcontrol-position: top right;
}
QComboBox#eventFilterCombo::down-arrow {
    margin-right: 2px;
}
QComboBox#eventFilterCombo QAbstractItemView {
    background-color: #283046;
    color: #eef3ff;
    border: 1px solid #3c4866;
    selection-background-color: #3b4766;
    selection-color: #ffffff;
}
QLineEdit#eventFilterSearch {
    background-color: #2c354a;
    color: #eef3ff;
    border: 1px solid #3c4866;
    border-radius: 8px;
    padding: 4px 10px;
    min-height: 22px;
}
QLineEdit#eventFilterSearch:focus {
    border-color: #6476a8;
}
QListWidget#listEvent {
    background-color: #1c2231;
    border: 1px solid #34415d;
    border-radius: 10px;
    padding: 6px;
    outline: none;
}
QListWidget#listEvent::item {
    background-color: #252d3f;
    border: 1px solid #33415e;
    border-radius: 8px;
    margin: 0 0 6px 0;
    padding: 8px 10px;
    color: #d9e2f7;
}
QListWidget#listEvent::item:selected {
    background-color: #32415f;
    border: 1px solid #5970a0;
    color: #ffffff;
}
QListWidget#listEvent::item:hover:!selected {
    background-color: #2b3750;
    border: 1px solid #455579;
}
QScrollBar:vertical {
    background: #171d2a;
    width: 10px;
    margin: 2px;
    border-radius: 5px;
}
QScrollBar::handle:vertical {
    background: #455474;
    min-height: 28px;
    border-radius: 5px;
}
QScrollBar::handle:vertical:hover {
    background: #5b6f98;
}
QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical,
QScrollBar::sub-page:vertical,
QScrollBar::left-arrow:vertical,
QScrollBar::right-arrow:vertical,
QScrollBar::up-arrow:vertical,
QScrollBar::down-arrow:vertical {
    height: 0px;
    background: transparent;
}
QScrollBar:horizontal {
    background: #171d2a;
    height: 10px;
    margin: 2px;
    border-radius: 5px;
}
QScrollBar::handle:horizontal {
    background: #455474;
    min-width: 28px;
    border-radius: 5px;
}
QScrollBar::handle:horizontal:hover {
    background: #5b6f98;
}
QScrollBar::add-line:horizontal,
QScrollBar::sub-line:horizontal,
QScrollBar::add-page:horizontal,
QScrollBar::sub-page:horizontal,
QScrollBar::left-arrow:horizontal,
QScrollBar::right-arrow:horizontal,
QScrollBar::up-arrow:horizontal,
QScrollBar::down-arrow:horizontal {
    width: 0px;
    background: transparent;
}
QStatusBar {
    background-color: #121621;
    color: #9da8cf;
}
)");

    if (ui->labelCentralView) ui->labelCentralView->setStyleSheet("background-color:black;");
    if (ui->labelTankView) {
        ui->labelTankView->setStyleSheet("background-color:black;");
        ui->labelTankView->setStreamProfile(GstVideoWidget::StreamProfile::RcTankLowLatency);
    }

    ui->textMsgLog->setReadOnly(true);
    ui->textMsgLog->setMaximumBlockCount(1000);
    ui->textMsgLog->setFont(QFont("Consolas", 10));
    if (ui->listEvent) {
        ui->listEvent->setSpacing(4);
        ui->listEvent->setAlternatingRowColors(false);
        ui->listEvent->setUniformItemSizes(false);
    }

    auto styleSmallToggle = [](QPushButton* b) {
        if (!b) return;        b->setMinimumHeight(20);
        b->setMaximumHeight(20);
        b->setCheckable(true);
        b->setAutoExclusive(false);
    };
    styleSmallToggle(findChild<QPushButton*>("btnCapture"));
    styleSmallToggle(findChild<QPushButton*>("btnMap"));
    styleSmallToggle(findChild<QPushButton*>("Humanbtn"));
    styleSmallToggle(findChild<QPushButton*>("ruviewbtn"));
    if (auto* b = findChild<QPushButton*>("btnCapture")) b->setText("Capture");
    if (auto* b = findChild<QPushButton*>("btnMap")) b->setText("Map");
    if (auto* b = findChild<QPushButton*>("btnRC")) b->setText("Node");
    if (auto* b = findChild<QPushButton*>("Humanbtn")) {
        b->setVisible(true);
        b->setText(QStringLiteral("?щ엺 媛먯?"));
        b->setMinimumHeight(38);
        b->setMaximumHeight(38);
        b->setMaximumWidth(QWIDGETSIZE_MAX);
    }
    if (auto* b = findChild<QPushButton*>("ruviewbtn")) {
        b->setMinimumHeight(38);
        b->setMaximumHeight(38);
        b->setMaximumWidth(QWIDGETSIZE_MAX);
    }
    if (auto* b = findChild<QPushButton*>("Humanbtn")) b->setText("Human Detection");
    if (auto* b = findChild<QPushButton*>("ruviewbtn")) b->setText("Blind Spot Detection");

    if (auto* btnCapture = findChild<QPushButton*>("btnCapture")) {
        btnCapture->setMinimumHeight(38);
        btnCapture->setMaximumHeight(38);
        btnCapture->setMaximumWidth(QWIDGETSIZE_MAX);
    }
    if (auto* btnMap = findChild<QPushButton*>("btnMap")) {
        btnMap->setMinimumHeight(38);
        btnMap->setMaximumHeight(38);
        btnMap->setMaximumWidth(QWIDGETSIZE_MAX);
    }
    if (auto* openAllBtn = findChild<QPushButton*>("btnOpenAllCctv")) {
        openAllBtn->setText("All CCTV");
        openAllBtn->setMinimumHeight(38);
        openAllBtn->setMaximumHeight(38);
        openAllBtn->setMaximumWidth(QWIDGETSIZE_MAX);
    }
    if (auto* patrolBtn = findChild<QPushButton*>("patrolstartbtn")) patrolBtn->setText("Patrol Start");
}

void MainWindow::buildResponsiveLayout()
{
    auto* root = ui ? ui->centralWidget : nullptr;
    if (!root) return;

    auto* panelCenter = root->findChild<QWidget*>("panelCenter");
    auto* paneDetailed = root->findChild<QWidget*>("paneDetailed");
    auto* panelRight = root->findChild<QWidget*>("panelRight");
    auto* groupCentral = root->findChild<QGroupBox*>("groupCentral");
    auto* groupWard = root->findChild<QGroupBox*>("groupWard");
    auto* tankCctv = root->findChild<QGroupBox*>("tankCctv");
    auto* groupMsg = root->findChild<QGroupBox*>("groupMsg");
    auto* barMain = root->findChild<QWidget*>("widgetCentralChBar");
    auto* barSide = root->findChild<QWidget*>("widgetCentralChBar_2");
    auto* oldRootHost = root->findChild<QWidget*>("horizontalLayoutWidget");
    if (!panelCenter || !paneDetailed || !panelRight || !groupCentral || !groupMsg) return;

    auto* centerLayout = new QVBoxLayout(panelCenter);
    centerLayout->setContentsMargins(2, 2, 2, 2);
    centerLayout->setSpacing(6);
    centerLayout->addWidget(groupCentral, 1);

    m_primaryNavList = new QListWidget(paneDetailed);
    m_primaryNavList->setObjectName("primaryNavList");
    m_primaryNavList->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    m_primaryNavList->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_primaryNavList->setSelectionMode(QAbstractItemView::SingleSelection);
    m_primaryNavList->setFocusPolicy(Qt::NoFocus);
    m_primaryNavList->setSpacing(2);
    m_primaryNavList->setFrameShape(QFrame::NoFrame);
    if (auto* b = findChild<QPushButton*>("btnCapture")) b->hide();
    if (auto* b = findChild<QPushButton*>("btnMap")) b->hide();
    if (auto* b = findChild<QPushButton*>("Humanbtn")) b->hide();
    if (auto* b = findChild<QPushButton*>("ruviewbtn")) b->hide();
    if (auto* b = findChild<QPushButton*>("btnOpenAllCctv")) b->hide();

    auto* centralLayout = new QVBoxLayout(groupCentral);
    centralLayout->setContentsMargins(8, 8, 8, 8);
    centralLayout->setSpacing(6);
    m_primaryViewStack = new QStackedWidget(groupCentral);
    m_primaryViewStack->setObjectName("primaryViewStack");
    if (ui->centralOverlayHost) {
        auto* centralCard = new QFrame(groupCentral);
        m_centralVideoCard = centralCard;
        centralCard->setObjectName("centralCard");
        centralCard->setProperty("visualPriority", "primary");
        centralCard->setFrameShape(QFrame::StyledPanel);
        auto* centralCardLayout = new QVBoxLayout(centralCard);
        centralCardLayout->setContentsMargins(8, 8, 8, 8);
        centralCardLayout->setSpacing(0);

        auto* aspectHost = new QWidget(centralCard);
        aspectHost->setObjectName("centralAspectHost");
        auto* aspectV = new QVBoxLayout(aspectHost);
        aspectV->setContentsMargins(0, 0, 0, 0);
        aspectV->setSpacing(0);
        auto* mid = new QHBoxLayout();
        mid->setContentsMargins(0, 0, 0, 0);
        mid->setSpacing(0);
        auto* leftSpacer = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
        auto* rightSpacer = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
        mid->addItem(leftSpacer);
        mid->addWidget(ui->centralOverlayHost, 0, Qt::AlignCenter);
        mid->addItem(rightSpacer);
        aspectV->addStretch(1);
        aspectV->addLayout(mid);
        aspectV->addStretch(1);
        centralCardLayout->addWidget(aspectHost, 1);
        m_primaryViewStack->addWidget(centralCard);
    }
    auto* tankCard = new QFrame(groupCentral);
    m_tankVideoCard = tankCard;
    tankCard->setObjectName("centralCard");
    tankCard->setProperty("visualPriority", "secondary");
    tankCard->setFrameShape(QFrame::StyledPanel);
    auto* tankLayout = new QVBoxLayout(tankCard);
    tankLayout->setContentsMargins(8, 8, 8, 8);
    tankLayout->setSpacing(6);
    m_tankHeaderHost = new QWidget(tankCard);
    m_tankHeaderHost->setObjectName("sideVideoHeaderHost");
    auto* tankHeaderRow = new QHBoxLayout(m_tankHeaderHost);
    tankHeaderRow->setContentsMargins(0, 0, 0, 0);
    tankHeaderRow->setSpacing(4);
    auto* tankHeader = new QLabel("Tank View", m_tankHeaderHost);
    tankHeader->setObjectName("sideVideoHeader");
    tankHeaderRow->addWidget(tankHeader, 1);
    tankLayout->addWidget(m_tankHeaderHost, 0);
    if (ui->labelTankView) {
        auto* tankAspectHost = new QWidget(tankCard);
        tankAspectHost->setObjectName("tankAspectHost");
        auto* v = new QVBoxLayout(tankAspectHost);
        v->setContentsMargins(0, 0, 0, 0);
        v->setSpacing(0);
        v->addStretch(1);
        auto* row = new QHBoxLayout();
        row->setContentsMargins(0, 0, 0, 0);
        row->setSpacing(0);
        auto* left = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
        auto* right = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
        row->addItem(left);
        row->addWidget(ui->labelTankView, 0, Qt::AlignCenter);
        row->addItem(right);
        v->addLayout(row);
        v->addStretch(1);
        tankLayout->addWidget(tankAspectHost, 1);
    }
    auto* tankControlCard = new QFrame(tankCard);
    tankControlCard->setObjectName("tankControlCard");
    auto* tankControlLayout = new QVBoxLayout(tankControlCard);
    tankControlLayout->setContentsMargins(12, 12, 12, 12);
    tankControlLayout->setSpacing(10);

    auto* tankControlHeader = new QHBoxLayout();
    tankControlHeader->setContentsMargins(0, 0, 0, 0);
    tankControlHeader->setSpacing(8);
    auto* tankControlTitle = new QLabel("Tank Controls", tankControlCard);
    tankControlTitle->setProperty("tankControlTitle", true);
    m_tankControlStatusLabel = new QLabel("Ready", tankControlCard);
    m_tankControlStatusLabel->setProperty("tankControlStatus", true);
    tankControlHeader->addWidget(tankControlTitle, 1);
    tankControlHeader->addWidget(m_tankControlStatusLabel, 0, Qt::AlignRight);
    tankControlLayout->addLayout(tankControlHeader);

    auto* tankControlBody = new QHBoxLayout();
    tankControlBody->setContentsMargins(0, 0, 0, 0);
    tankControlBody->setSpacing(12);

    auto* driveCard = new QFrame(tankControlCard);
    auto* driveLayout = new QVBoxLayout(driveCard);
    driveLayout->setContentsMargins(0, 0, 0, 0);
    driveLayout->setSpacing(6);
    auto* driveTitle = new QLabel("Drive", driveCard);
    driveTitle->setProperty("tankControlStatus", true);
    driveLayout->addWidget(driveTitle, 0, Qt::AlignLeft);
    auto* driveGrid = new QGridLayout();
    driveGrid->setContentsMargins(0, 0, 0, 0);
    driveGrid->setHorizontalSpacing(8);
    driveGrid->setVerticalSpacing(8);
    auto* btnForward = new QPushButton("FORWARD", driveCard);
    auto* btnBackward = new QPushButton("REVERSE", driveCard);
    auto* btnTurnLeft = new QPushButton("TURN LEFT", driveCard);
    auto* btnTurnRight = new QPushButton("TURN RIGHT", driveCard);
    const auto driveButtons = {btnForward, btnBackward, btnTurnLeft, btnTurnRight};
    for (auto* button : driveButtons) {
        button->setProperty("tankControlButton", true);
        button->setProperty("tankGroup", "drive");
    }
    driveGrid->addWidget(btnForward, 0, 1);
    driveGrid->addWidget(btnTurnLeft, 1, 0);
    driveGrid->addWidget(btnBackward, 1, 1);
    driveGrid->addWidget(btnTurnRight, 1, 2);
    driveLayout->addLayout(driveGrid);
    tankControlBody->addWidget(driveCard, 1);

    auto* ptzCard = new QFrame(tankControlCard);
    auto* ptzLayout = new QVBoxLayout(ptzCard);
    ptzLayout->setContentsMargins(0, 0, 0, 0);
    ptzLayout->setSpacing(6);
    auto* ptzTitle = new QLabel("Pan / Tilt", ptzCard);
    ptzTitle->setProperty("tankControlStatus", true);
    ptzLayout->addWidget(ptzTitle, 0, Qt::AlignLeft);
    auto* ptzGrid = new QGridLayout();
    ptzGrid->setContentsMargins(0, 0, 0, 0);
    ptzGrid->setHorizontalSpacing(8);
    ptzGrid->setVerticalSpacing(8);
    auto* btnTiltUp = new QPushButton("TILT UP", ptzCard);
    auto* btnTiltDown = new QPushButton("TILT DOWN", ptzCard);
    auto* btnPanLeft = new QPushButton("PAN LEFT", ptzCard);
    auto* btnPanRight = new QPushButton("PAN RIGHT", ptzCard);
    const auto ptzButtons = {btnTiltUp, btnTiltDown, btnPanLeft, btnPanRight};
    for (auto* button : ptzButtons) {
        button->setProperty("tankControlButton", true);
        button->setProperty("tankGroup", "ptz");
    }
    ptzGrid->addWidget(btnTiltUp, 0, 1);
    ptzGrid->addWidget(btnPanLeft, 1, 0);
    ptzGrid->addWidget(btnTiltDown, 1, 1);
    ptzGrid->addWidget(btnPanRight, 1, 2);
    ptzLayout->addLayout(ptzGrid);
    tankControlBody->addWidget(ptzCard, 1);

    tankControlLayout->addLayout(tankControlBody);
    tankLayout->addWidget(tankControlCard, 0);

    bindTankControlButton(btnForward, "drive", "forward");
    bindTankControlButton(btnBackward, "drive", "backward");
    bindTankControlButton(btnTurnLeft, "drive", "turn_left");
    bindTankControlButton(btnTurnRight, "drive", "turn_right");
    bindTankControlButton(btnTiltUp, "ptz", "tilt_up");
    bindTankControlButton(btnTiltDown, "ptz", "tilt_down");
    bindTankControlButton(btnPanLeft, "ptz", "pan_left");
    bindTankControlButton(btnPanRight, "ptz", "pan_right");

    m_primaryViewStack->addWidget(tankCard);
    centralLayout->addWidget(m_primaryViewStack, 1);

    auto* detailLayout = new QVBoxLayout(paneDetailed);
    detailLayout->setContentsMargins(2, 2, 2, 2);
    detailLayout->setSpacing(6);
    auto* navHeader = new QFrame(paneDetailed);
    navHeader->setObjectName("navPanelHeader");
    auto* navHeaderLayout = new QVBoxLayout(navHeader);
    navHeaderLayout->setContentsMargins(14, 12, 14, 12);
    navHeaderLayout->setSpacing(2);
    auto* navEyebrow = new QLabel("OPERATIONS", navHeader);
    navEyebrow->setObjectName("navPanelEyebrow");
    auto* navTitle = new QLabel("Sentinel Control", navHeader);
    navTitle->setObjectName("navPanelTitle");
    auto* navSubtitle = new QLabel("Mode and action navigation", navHeader);
    navSubtitle->setObjectName("navPanelSubtitle");
    navHeaderLayout->addWidget(navEyebrow);
    navHeaderLayout->addWidget(navTitle);
    navHeaderLayout->addWidget(navSubtitle);
    auto applyShadow = [](QWidget* w, const QColor& color = QColor(25, 32, 52, 55)) {
        if (!w) return;
        auto* shadow = new QGraphicsDropShadowEffect(w);
        shadow->setBlurRadius(28);
        shadow->setOffset(0, 10);
        shadow->setColor(color);
        w->setGraphicsEffect(shadow);
    };
    applyShadow(navHeader, QColor(49, 64, 104, 45));
    applyShadow(m_primaryNavList, QColor(49, 64, 104, 35));
    detailLayout->addWidget(navHeader, 0);
    if (m_primaryNavList) detailLayout->addWidget(m_primaryNavList, 1);
    if (groupWard) {
        groupWard->setTitle(QString());
        groupWard->setMinimumHeight(138);
        groupWard->setMaximumHeight(156);
        detailLayout->addWidget(groupWard, 0);
    }
    m_robotStatusCard = new QFrame(paneDetailed);
    m_robotStatusCard->setObjectName("robotStatusCard");
    auto* robotLayout = new QVBoxLayout(m_robotStatusCard);
    robotLayout->setContentsMargins(12, 12, 12, 12);
    robotLayout->setSpacing(8);
    auto* robotHeaderRow = new QHBoxLayout();
    robotHeaderRow->setContentsMargins(0, 0, 0, 0);
    robotHeaderRow->setSpacing(8);
    auto* robotTitle = new QLabel("Tank Status", m_robotStatusCard);
    robotTitle->setProperty("robotTitle", true);
    auto* robotOpenLabel = new QLabel("OPEN >", m_robotStatusCard);
    robotOpenLabel->setProperty("robotOpen", true);
    robotHeaderRow->addWidget(robotTitle, 1);
    robotHeaderRow->addWidget(robotOpenLabel, 0, Qt::AlignRight);
    robotLayout->addLayout(robotHeaderRow);
    auto* connRow = new QHBoxLayout();
    auto* connLabel = new QLabel("Connection", m_robotStatusCard);
    connLabel->setProperty("robotLabel", true);
    connLabel->setFixedWidth(66);
    m_robotConnLabel = new QLabel("OFFLINE", m_robotStatusCard);
    m_robotConnLabel->setProperty("robotValue", true);
    m_robotConnLabel->setProperty("robotConn", "offline");
    m_robotConnLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    connRow->addWidget(connLabel);
    connRow->addStretch();
    connRow->addWidget(m_robotConnLabel);
    robotLayout->addLayout(connRow);
    auto* modeRow = new QHBoxLayout();
    auto* modeLabel = new QLabel("Mode", m_robotStatusCard);
    modeLabel->setProperty("robotLabel", true);
    modeLabel->setFixedWidth(66);
    m_robotModeLabel = new QLabel("-", m_robotStatusCard);
    m_robotModeLabel->setProperty("robotValue", true);
    m_robotModeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    modeRow->addWidget(modeLabel);
    modeRow->addStretch();
    modeRow->addWidget(m_robotModeLabel);
    robotLayout->addLayout(modeRow);
    auto* missionRow = new QHBoxLayout();
    auto* missionLabel = new QLabel("Mission", m_robotStatusCard);
    missionLabel->setProperty("robotLabel", true);
    missionLabel->setFixedWidth(66);
    m_robotMissionLabel = new QLabel("-", m_robotStatusCard);
    m_robotMissionLabel->setProperty("robotValue", true);
    m_robotMissionLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    missionRow->addWidget(missionLabel);
    missionRow->addStretch();
    missionRow->addWidget(m_robotMissionLabel);
    robotLayout->addLayout(missionRow);
    auto* poseRow = new QHBoxLayout();
    auto* poseLabel = new QLabel("Pose", m_robotStatusCard);
    poseLabel->setProperty("robotLabel", true);
    poseLabel->setFixedWidth(66);
    m_robotPoseLabel = new QLabel("x - / y - / h -", m_robotStatusCard);
    m_robotPoseLabel->setProperty("robotValue", true);
    m_robotPoseLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    poseRow->addWidget(poseLabel);
    poseRow->addStretch();
    poseRow->addWidget(m_robotPoseLabel);
    robotLayout->addLayout(poseRow);
    auto* speedRow = new QHBoxLayout();
    auto* speedLabel = new QLabel("Speed", m_robotStatusCard);
    speedLabel->setProperty("robotLabel", true);
    speedLabel->setFixedWidth(66);
    m_robotSpeedLabel = new QLabel("- m/s", m_robotStatusCard);
    m_robotSpeedLabel->setProperty("robotValue", true);
    m_robotSpeedLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    speedRow->addWidget(speedLabel);
    speedRow->addStretch();
    speedRow->addWidget(m_robotSpeedLabel);
    robotLayout->addLayout(speedRow);
    auto* cpuRow = new QHBoxLayout();
    auto* cpuLabel = new QLabel("CPU", m_robotStatusCard);
    cpuLabel->setProperty("robotLabel", true);
    cpuLabel->setFixedWidth(66);
    m_robotCpuValueLabel = new QLabel("- %", m_robotStatusCard);
    m_robotCpuValueLabel->setProperty("robotValue", true);
    m_robotCpuValueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    cpuRow->addWidget(cpuLabel);
    cpuRow->addStretch();
    cpuRow->addWidget(m_robotCpuValueLabel);
    robotLayout->addLayout(cpuRow);
    auto* lastSeenRow = new QHBoxLayout();
    auto* lastSeenLabel = new QLabel("Last Seen", m_robotStatusCard);
    lastSeenLabel->setProperty("robotLabel", true);
    lastSeenLabel->setFixedWidth(66);
    m_robotLastSeenLabel = new QLabel("-", m_robotStatusCard);
    m_robotLastSeenLabel->setProperty("robotValue", true);
    m_robotLastSeenLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    lastSeenRow->addWidget(lastSeenLabel);
    lastSeenRow->addStretch();
    lastSeenRow->addWidget(m_robotLastSeenLabel);
    robotLayout->addLayout(lastSeenRow);
    const auto robotChildren = m_robotStatusCard->findChildren<QWidget*>();
    for (QWidget* child : robotChildren) {
        if (child && child != m_robotStatusCard) {
            child->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        }
    }
    m_robotStatusCard->setCursor(Qt::PointingHandCursor);
    m_robotStatusCard->setToolTip("Open robot dashboard");
    m_robotStatusCard->installEventFilter(this);
    m_robotStatusCard->setMinimumHeight(164);
    m_robotStatusCard->setMaximumHeight(210);
    detailLayout->addWidget(m_robotStatusCard, 0);
    applyShadow(groupCentral, QColor(25, 32, 52, 40));
    applyShadow(groupMsg, QColor(25, 32, 52, 35));
    applyShadow(m_centralVideoCard, QColor(25, 32, 52, 45));
    applyShadow(m_tankVideoCard, QColor(25, 32, 52, 45));
    applyShadow(m_robotStatusCard, QColor(25, 32, 52, 35));

    auto* panelRightLayout = new QVBoxLayout(panelRight);
    panelRightLayout->setContentsMargins(2, 2, 2, 2);
    panelRightLayout->setSpacing(6);
    panelRightLayout->addWidget(groupMsg, 1);

    auto* msgLayout = new QVBoxLayout(groupMsg);
    msgLayout->setObjectName("verticalLayout_msg");
    msgLayout->setContentsMargins(10, 10, 10, 10);
    msgLayout->setSpacing(8);
    if (ui->listEvent) {
        ui->listEvent->setMinimumHeight(280);
        msgLayout->addWidget(ui->listEvent, 1);
    }
    if (ui->textMsgLog) {
        ui->textMsgLog->setMinimumHeight(96);
        ui->textMsgLog->setMaximumHeight(148);
        msgLayout->addWidget(ui->textMsgLog, 0);
    }

    if (groupWard) {
        m_systemUsageCard = groupWard;
        auto* wardLayout = new QVBoxLayout(groupWard);
        wardLayout->setContentsMargins(10, 10, 10, 10);
        wardLayout->setSpacing(8);

        auto* usageTitleRow = new QHBoxLayout();
        usageTitleRow->setContentsMargins(0, 0, 0, 0);
        usageTitleRow->setSpacing(6);
        auto* usageTitle = new QLabel("System Usage", groupWard);
        usageTitle->setProperty("robotTitle", true);
        usageTitleRow->addWidget(usageTitle, 1, Qt::AlignVCenter);
        wardLayout->addLayout(usageTitleRow);

        auto* usageMetrics = new QHBoxLayout();
        usageMetrics->setContentsMargins(0, 0, 0, 0);
        usageMetrics->setSpacing(8);
        const auto addUsageMetric = [groupWard, usageMetrics](const QString& label, QLabel** outLabel) {
            auto* metricCard = new QFrame(groupWard);
            metricCard->setProperty("statusMetricCard", true);
            auto* metricLayout = new QVBoxLayout(metricCard);
            metricLayout->setContentsMargins(10, 8, 10, 8);
            metricLayout->setSpacing(2);
            auto* metricLabel = new QLabel(label, metricCard);
            metricLabel->setProperty("statusMetricLabel", true);
            auto* metricValue = new QLabel("-", metricCard);
            metricValue->setProperty("statusMetricValue", true);
            metricLayout->addWidget(metricLabel);
            metricLayout->addWidget(metricValue);
            usageMetrics->addWidget(metricCard, 1);
            if (outLabel) *outLabel = metricValue;
        };
        addUsageMetric("CPU", &m_systemCpuValueLabel);
        addUsageMetric("MEM", &m_systemMemoryValueLabel);
        addUsageMetric("GPU", &m_systemGpuValueLabel);
        wardLayout->addLayout(usageMetrics);

        if (ui->listWardCctv) {
            ui->listWardCctv->hide();
            wardLayout->addWidget(ui->listWardCctv, 0);
        }
        const auto usageChildren = groupWard->findChildren<QWidget*>();
        for (QWidget* child : usageChildren) {
            if (child && child != groupWard) {
                child->setAttribute(Qt::WA_TransparentForMouseEvents, true);
            }
        }
        groupWard->setCursor(Qt::PointingHandCursor);
        groupWard->setToolTip("Open system usage dashboard");
        groupWard->installEventFilter(this);
    }
    if (tankCctv) tankCctv->hide();
    if (auto* patrolCctv = root->findChild<QGroupBox*>("patrolCctv")) {
        patrolCctv->hide();
    }
    if (auto* patrolView = findChild<QWidget*>("labelPatrolView")) {
        patrolView->hide();
    }

    auto* rootLayout = new QVBoxLayout(root);
    rootLayout->setContentsMargins(4, 4, 4, 4);
    rootLayout->setSpacing(4);

    auto* nameBar = new QWidget(root);
    nameBar->setObjectName("platformNameBar");
    nameBar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    nameBar->setFixedHeight(34);
    auto* nameBarLayout = new QHBoxLayout(nameBar);
    nameBarLayout->setContentsMargins(10, 4, 10, 4);
    nameBarLayout->setSpacing(8);
    m_headerModeLabel = new QLabel("MAP", nameBar);
    m_headerModeLabel->setProperty("headerMeta", true);
    m_headerUserLabel = new QLabel("CTRL", nameBar);
    m_headerUserLabel->setProperty("headerMeta", true);
    auto* platformLabel = new QLabel("Sentinel Fusion", nameBar);
    platformLabel->setObjectName("platformNameLabel");
    platformLabel->setAlignment(Qt::AlignCenter);
    QFont platformFont = platformLabel->font();
    platformFont.setPointSize(12);
    platformFont.setBold(true);
    platformLabel->setFont(platformFont);
    m_headerRcStatusLabel = new QLabel("RC WAIT", nameBar);
    m_headerRcStatusLabel->setProperty("headerMeta", true);
    m_headerRuviewStatusLabel = new QLabel("RUVIEW OFF", nameBar);
    m_headerRuviewStatusLabel->setProperty("headerMeta", true);
    m_headerRuviewStatusLabel->setProperty("accent", "status");
    nameBarLayout->addWidget(m_headerModeLabel, 0, Qt::AlignVCenter);
    nameBarLayout->addWidget(m_headerUserLabel, 0, Qt::AlignVCenter);
    nameBarLayout->addWidget(platformLabel, 1, Qt::AlignCenter);
    nameBarLayout->addWidget(m_headerRcStatusLabel, 0, Qt::AlignVCenter);
    nameBarLayout->addWidget(m_headerRuviewStatusLabel, 0, Qt::AlignVCenter);

    m_mainSplitter = new QSplitter(Qt::Horizontal, root);
    m_mainSplitter->setObjectName("mainSplitter");
    m_mainSplitter->setHandleWidth(2);
    m_mainSplitter->addWidget(paneDetailed);
    m_mainSplitter->addWidget(panelCenter);
    m_mainSplitter->addWidget(panelRight);
    m_mainSplitter->setStretchFactor(0, 2);
    m_mainSplitter->setStretchFactor(1, 7);
    m_mainSplitter->setStretchFactor(2, 4);
    m_mainSplitter->setCollapsible(0, false);
    m_mainSplitter->setCollapsible(1, false);
    m_mainSplitter->setCollapsible(2, false);

    panelCenter->setMinimumWidth(780);
    paneDetailed->setMinimumWidth(248);
    panelRight->setMinimumWidth(360);
    paneDetailed->setMaximumWidth(300);
    panelRight->setMaximumWidth(500);

    rootLayout->addWidget(nameBar, 0);
    rootLayout->addWidget(m_mainSplitter, 1);

    if (oldRootHost) {
        oldRootHost->hide();
        oldRootHost->deleteLater();
    }
    if (barMain) barMain->hide();
    if (barSide) barSide->hide();
    if (m_primaryNavList) {
        connect(m_primaryNavList, &QListWidget::itemClicked,
                this, &MainWindow::onPrimaryNavItemClicked);
    }
    applyPrimaryViewMode(PrimaryViewMode::Map);
    applyMainSplitterDefaultSizes();
    updateCentralAspectRatio();
    updateSideVideoAspectRatios();
    updateHeaderSummary();
}

void MainWindow::updatePrimaryModeControls()
{
    if (auto* b = findChild<QPushButton*>("btnCapture")) b->hide();
    if (auto* b = findChild<QPushButton*>("btnMap")) b->hide();
    if (auto* b = findChild<QPushButton*>("btnRC")) b->hide();
    if (auto* b = findChild<QPushButton*>("Humanbtn")) b->hide();
    if (auto* b = findChild<QPushButton*>("ruviewbtn")) b->hide();
    if (auto* b = findChild<QPushButton*>("btnOpenAllCctv")) b->hide();
    rebuildPrimaryNavList();
}

void MainWindow::rebuildPrimaryNavList()
{
    if (!m_primaryNavList) return;

    m_primaryNavList->clear();
    const auto addItem = [this](const QString& text, const QString& token, const QFont& font) {
        auto* item = new QListWidgetItem(text, m_primaryNavList);
        item->setData(Qt::UserRole, token);
        item->setFont(font);
        item->setSizeHint(QSize(0, font.bold() ? 96 : 70));
        item->setText(QString());
        return item;
    };
    const auto decorateItem = [this](QListWidgetItem* item, const QString& title, const QString& subtitle,
                                      const QString& badge, const QString& modeKey,
                                      bool selected, bool action) {
        if (!item) return;
        auto* outer = new QFrame(m_primaryNavList);
        outer->setProperty("navGlow", true);
        outer->setProperty("selected", selected);
        auto* outerLayout = new QVBoxLayout(outer);
        outerLayout->setContentsMargins(0, 0, 0, 0);
        outerLayout->setSpacing(0);
        auto* card = new QFrame(outer);
        card->setProperty("navItemFrame", true);
        card->setProperty("selected", selected);
        card->setProperty("action", action);
        outerLayout->addWidget(card);
        auto* row = new QHBoxLayout(card);
        row->setContentsMargins(action ? 12 : 14, action ? 9 : 12, 14, action ? 9 : 12);
        row->setSpacing(12);

        if (!action) {
            auto* icon = new QLabel(modeKey == "map" ? "M"
                                     : modeKey == "detection" ? "D"
                                     : modeKey == "ruview" ? "R"
                                     : modeKey == "tank" ? "T" : "A", card);
            icon->setProperty("navIcon", true);
            icon->setProperty("mode", modeKey);
            row->addWidget(icon, 0, Qt::AlignVCenter);
        }

        if (selected && !action) {
            auto* indicator = new QLabel(card);
            indicator->setProperty("navIndicator", true);
            indicator->setProperty("mode", modeKey);
            indicator->setFixedSize(6, 28);
            row->addWidget(indicator, 0, Qt::AlignVCenter);
        } else if (!action) {
            row->addSpacing(6);
        } else {
            row->addSpacing(18);
        }

        auto* textCol = new QVBoxLayout();
        textCol->setContentsMargins(0, 0, 0, 0);
        textCol->setSpacing(subtitle.isEmpty() ? 0 : 4);
        auto* titleLabel = new QLabel(title, card);
        titleLabel->setProperty("navTitle", true);
        titleLabel->setProperty("action", action);
        titleLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        textCol->addWidget(titleLabel);
        if (!subtitle.isEmpty()) {
            auto* subtitleLabel = new QLabel(subtitle, card);
            subtitleLabel->setProperty("navSubtitle", true);
            subtitleLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
            textCol->addWidget(subtitleLabel);
        }
        row->addLayout(textCol, 1);

        if (!badge.isEmpty()) {
            auto* badgeLabel = new QLabel(badge, card);
            badgeLabel->setProperty("navBadge", true);
            badgeLabel->setProperty("mode", modeKey);
            badgeLabel->setProperty("active", badge == "ON");
            row->addWidget(badgeLabel, 0, Qt::AlignVCenter);
        }

        m_primaryNavList->setItemWidget(item, outer);
    };

    QFont modeFont = font();
    modeFont.setPointSize(std::max(10, modeFont.pointSize()));
    modeFont.setBold(true);
    QFont actionFont = modeFont;
    actionFont.setBold(false);
    actionFont.setPointSize(std::max(9, modeFont.pointSize() - 1));

    const auto modeToken = [](PrimaryViewMode mode) {
        switch (mode) {
        case PrimaryViewMode::Map: return QString("mode:map");
        case PrimaryViewMode::Detection: return QString("mode:detection");
        case PrimaryViewMode::RuView: return QString("mode:ruview");
        case PrimaryViewMode::Tank: return QString("mode:tank");
        }
        return QString("mode:map");
    };

    const QList<QPair<QString, PrimaryViewMode>> modes = {
        {"Map", PrimaryViewMode::Map},
        {"Detection", PrimaryViewMode::Detection},
        {"RuView", PrimaryViewMode::RuView},
        {"Tank", PrimaryViewMode::Tank},
    };

    for (const auto& entry : modes) {
        const bool selected = (m_primaryViewMode == entry.second);
        auto* modeItem = addItem(entry.first, modeToken(entry.second), modeFont);
        const QString subtitle =
            (entry.second == PrimaryViewMode::Map) ? "Map overlays and capture" :
            (entry.second == PrimaryViewMode::Detection) ? "Human detection tools" :
            (entry.second == PrimaryViewMode::RuView) ? "Blind spot monitoring" :
            "Tank camera workspace";
        QString badge;
        if (entry.second == PrimaryViewMode::Map) {
            badge = m_mapNavExpanded ? "v" : ">";
        } else if (entry.second == PrimaryViewMode::Detection) {
            badge = m_humanBoxEnabled ? "ON" : "OFF";
        } else if (entry.second == PrimaryViewMode::RuView) {
            badge = m_ruviewUiEnabled ? "ON" : "OFF";
        }
        const QString modeKey =
            (entry.second == PrimaryViewMode::Map) ? "map" :
            (entry.second == PrimaryViewMode::Detection) ? "detection" :
            (entry.second == PrimaryViewMode::RuView) ? "ruview" : "tank";
        decorateItem(modeItem, entry.first, subtitle, badge, modeKey, selected, false);
        if (selected) {
            modeItem->setData(Qt::ForegroundRole, QColor("#1f2a44"));
            m_primaryNavList->setCurrentItem(modeItem);
        } else {
            modeItem->setData(Qt::ForegroundRole, QColor("#5f6b84"));
        }
        if (!selected) continue;

        if (entry.second == PrimaryViewMode::Map && m_mapNavExpanded) {
            auto* a1 = addItem("  Capture", "action:btnCapture", actionFont);
            auto* a2 = addItem("  Map", "action:btnMap", actionFont);
            auto* a3 = addItem("  Node", "action:btnRC", actionFont);
            decorateItem(a1, "Capture", "Save calibration frame", "", "map", m_activePrimaryNavActionToken == "action:btnCapture", true);
            decorateItem(a2, "Map Viewer", "Toggle map overlay", m_mapOverlayEnabled ? "ON" : "OFF", "map", m_activePrimaryNavActionToken == "action:btnMap", true);
            decorateItem(a3, "Node", "Send RC goal point", m_cameraGoalMode ? "ON" : "OFF", "map", m_activePrimaryNavActionToken == "action:btnRC", true);
        }
    }

    auto* allCctvItem = addItem("All CCTV", "action:btnOpenAllCctv", modeFont);
    const bool allSelected = (m_activePrimaryNavActionToken == "action:btnOpenAllCctv");
    decorateItem(allCctvItem, "All CCTV", "Open full camera wall", "", "allcctv", allSelected, false);
    if (allSelected) {
        m_primaryNavList->setCurrentItem(allCctvItem);
    }
}

void MainWindow::onPrimaryNavItemClicked(QListWidgetItem* item)
{
    if (!item) return;
    const QString token = item->data(Qt::UserRole).toString();
    if (token == "mode:map") {
        if (m_primaryViewMode == PrimaryViewMode::Map) {
            m_mapNavExpanded = !m_mapNavExpanded;
            if (!m_mapNavExpanded) m_activePrimaryNavActionToken.clear();
        } else {
            m_mapNavExpanded = true;
            m_activePrimaryNavActionToken.clear();
            applyPrimaryViewMode(PrimaryViewMode::Map);
        }
        saveUiSettings();
        rebuildPrimaryNavList();
    } else if (token == "mode:detection") {
        m_activePrimaryNavActionToken = "action:Humanbtn";
        applyPrimaryViewMode(PrimaryViewMode::Detection);
        if (auto* b = findChild<QPushButton*>("Humanbtn")) b->click();
        rebuildPrimaryNavList();
    } else if (token == "mode:ruview") {
        m_activePrimaryNavActionToken = "action:ruviewbtn";
        applyPrimaryViewMode(PrimaryViewMode::RuView);
        if (auto* b = findChild<QPushButton*>("ruviewbtn")) b->click();
        rebuildPrimaryNavList();
    } else if (token == "mode:tank") {
        m_activePrimaryNavActionToken.clear();
        applyPrimaryViewMode(PrimaryViewMode::Tank);
    } else if (token == "action:btnCapture") {
        m_activePrimaryNavActionToken = token;
        if (auto* b = findChild<QPushButton*>("btnCapture")) b->click();
        rebuildPrimaryNavList();
    } else if (token == "action:btnMap") {
        m_activePrimaryNavActionToken = token;
        if (auto* b = findChild<QPushButton*>("btnMap")) b->click();
        rebuildPrimaryNavList();
    } else if (token == "action:btnRC") {
        m_activePrimaryNavActionToken = token;
        if (auto* b = findChild<QPushButton*>("btnRC")) b->click();
        rebuildPrimaryNavList();
    } else if (token == "action:Humanbtn") {
        m_activePrimaryNavActionToken = token;
        if (auto* b = findChild<QPushButton*>("Humanbtn")) b->click();
        rebuildPrimaryNavList();
    } else if (token == "action:ruviewbtn") {
        m_activePrimaryNavActionToken = token;
        if (auto* b = findChild<QPushButton*>("ruviewbtn")) b->click();
        rebuildPrimaryNavList();
    } else if (token == "action:btnOpenAllCctv") {
        m_activePrimaryNavActionToken = token;
        onOpenAllCctv();
        rebuildPrimaryNavList();
    }
}

void MainWindow::applyPrimaryViewMode(PrimaryViewMode mode)
{
    m_primaryViewMode = mode;

    if (m_primaryViewStack) {
        const int page = (mode == PrimaryViewMode::Tank) ? 1 : 0;
        if (m_primaryViewStack->currentIndex() != page) {
            m_primaryViewStack->setCurrentIndex(page);
        }
    }

    if (mode == PrimaryViewMode::Tank) {
        appendLog(QString("Primary mode -> Tank, cached url=%1")
                      .arg(m_tankPlayingUrl.isEmpty() ? "<empty>" : m_tankPlayingUrl));
        if (ui && ui->labelCentralView) {
            ui->labelCentralView->stopStream();
        }
        if (ui && ui->labelTankView && !m_tankPlayingUrl.isEmpty()) {
            appendLog(QString("Tank startStream -> %1").arg(m_tankPlayingUrl));
            ui->labelTankView->startStream(m_tankPlayingUrl);
        } else {
            appendLog("Tank startStream skipped: labelTankView missing or URL empty");
        }
    } else {
        if (ui && ui->labelTankView) {
            appendLog("Tank stopStream (leaving tank mode)");
            ui->labelTankView->stopStream();
        }
        if (ui && ui->labelCentralView && !m_centralPlayingUrl.isEmpty()) {
            ui->labelCentralView->startStream(m_centralPlayingUrl);
        }
    }

    updatePrimaryModeControls();
    updateCentralAspectRatio();
    updateSideVideoAspectRatios();
    syncCentralOverlayGeometry();
    updateCentralLayerOrder();
    updateHeaderSummary();

    QTimer::singleShot(0, this, [this]() {
        if (isMaximized() || isFullScreen()) return;

        QScreen* targetScreen = screen();
        if (!targetScreen) targetScreen = QGuiApplication::primaryScreen();
        if (!targetScreen) return;

        const QRect avail = targetScreen->availableGeometry();
        QRect g = geometry();
        bool changed = false;

        if (g.height() > avail.height()) {
            g.setHeight(avail.height());
            changed = true;
        }
        if (g.bottom() > avail.bottom()) {
            g.moveBottom(avail.bottom());
            changed = true;
        }
        if (g.top() < avail.top()) {
            g.moveTop(avail.top());
            changed = true;
        }

        if (changed) setGeometry(g);
    });
}

void MainWindow::updateHeaderSummary()
{
    const QString modeText =
        (m_primaryViewMode == PrimaryViewMode::Map) ? "MAP" :
        (m_primaryViewMode == PrimaryViewMode::Detection) ? "DETECT" :
        (m_primaryViewMode == PrimaryViewMode::RuView) ? "RUVIEW" :
        "TANK";
    const QString roleText = (m_role == UserRole::Executive) ? "EXEC" : "CTRL";
    QString userText = m_userId.isEmpty() ? roleText : m_userId.toUpper();
    if (userText.size() > 12) userText = userText.left(12);
    const QString robotText =
        m_lastRobotStatusEvent.hasRobotStatus
            ? QString("RC %1").arg(m_lastRobotStatusEvent.rcConnected ? "ONLINE" : "OFFLINE")
            : QString("RC WAIT");
    const QString ruviewText = m_ruviewUiEnabled ? "RUVIEW ON" : "RUVIEW OFF";
    if (m_headerModeLabel) m_headerModeLabel->setText(modeText);
    if (m_headerUserLabel) m_headerUserLabel->setText(QString("%1 | %2").arg(userText, roleText));
    if (m_headerRcStatusLabel) {
        m_headerRcStatusLabel->setText(robotText);
        m_headerRcStatusLabel->setProperty("accent", m_lastRobotStatusEvent.rcConnected ? "status" : "danger");
        m_headerRcStatusLabel->style()->unpolish(m_headerRcStatusLabel);
        m_headerRcStatusLabel->style()->polish(m_headerRcStatusLabel);
    }
    if (m_headerRuviewStatusLabel) {
        m_headerRuviewStatusLabel->setText(ruviewText);
    }
}

void MainWindow::initCentralStreamMap()
{
    // 以묒븰 梨꾨꼸 踰꾪듉(centralcctv)??
    m_centralRtsp[1] = "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
}

void MainWindow::initStreamMap()
{
    // ???占쎈━?占쎌꽌 ?占쎌젣占?議댁옱?占쎈뒗 ?占쏀듃由쇰쭔 ?占쎈줉 (六μ뭅??誘몃벑占?
    m_wardStreams.clear();

    // Main1 -> CentralView (?占쎌젣)
    //m_wardStreams["Main1"] = { "rtsp://192.168.100.8:8554/live", QString() };

    // T1 -> TankView (?占쎌젣)
    m_wardStreams["T1"]    = { "rtsp://192.168.100.5:8555/cam", QString() };
    appendLog(QString("Tank stream registered: T1 -> %1").arg(m_wardStreams["T1"].rawRtsp));

    // P1 -> PatrolView (?占쎌젣)

    // Main2/Main3/T2/T3/P2/P3 ???占쏙옙????占쎈줉?占쏙옙? ?占쎌쓬 (= ?占쏙옙?/六μ뭅)

    // Preselect the default tank stream so Tank view can start immediately.
    appendLog("Selecting default tank stream: T1");
    switchTankStreamTo("T1");
}

void MainWindow::initWardTree()
{
    ui->listWardCctv->clear();

    ui->listWardCctv->setColumnCount(1);
    ui->listWardCctv->setHeaderHidden(true);
    ui->listWardCctv->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui->listWardCctv->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->listWardCctv->setRootIsDecorated(false);
    ui->listWardCctv->setIndentation(12);

    m_systemUsageRoot = new QTreeWidgetItem(ui->listWardCctv);
    m_systemUsageRoot->setText(0, "System Usage");
    m_cpuUsageItem = new QTreeWidgetItem(m_systemUsageRoot);
    m_cpuUsageItem->setText(0, "CPU  -");
    m_memoryUsageItem = new QTreeWidgetItem(m_systemUsageRoot);
    m_memoryUsageItem->setText(0, "Memory  -");
    m_gpuUsageItem = new QTreeWidgetItem(m_systemUsageRoot);
    m_gpuUsageItem->setText(0, "GPU  -");

    m_systemUsageRoot->setFlags(m_systemUsageRoot->flags() & ~Qt::ItemIsSelectable);
    ui->listWardCctv->expandAll();
    if (m_cpuUsageItem) ui->listWardCctv->setCurrentItem(m_cpuUsageItem);
}
void MainWindow::updateSystemUsageTree()
{
    if (!m_cpuUsageItem || !m_memoryUsageItem || !m_gpuUsageItem) return;
    const auto applyUsageColor = [](QTreeWidgetItem* item, double value) {
        if (!item) return;
        QColor color("#dce3ff");
        if (value >= 85.0) color = QColor("#ff8d8d");
        else if (value >= 65.0) color = QColor("#f3c86b");
        item->setForeground(0, QBrush(color));
    };
    const auto applyMetricLevel = [](QLabel* label, double value, const QString& text) {
        if (!label) return;
        label->setText(text);
        QString level;
        if (value >= 85.0) level = "danger";
        else if (value >= 65.0) level = "warn";
        label->setProperty("level", level);
        label->style()->unpolish(label);
        label->style()->polish(label);
    };

#ifdef Q_OS_WIN
    FILETIME idleTime{}, kernelTime{}, userTime{};
    if (GetSystemTimes(&idleTime, &kernelTime, &userTime)) {
        const quint64 idle = fileTimeToUInt64(idleTime);
        const quint64 kernel = fileTimeToUInt64(kernelTime);
        const quint64 user = fileTimeToUInt64(userTime);

        QString cpuText = "CPU  -";
        if (m_hasCpuSample) {
            const quint64 idleDiff = idle - m_prevCpuIdle;
            const quint64 kernelDiff = kernel - m_prevCpuKernel;
            const quint64 userDiff = user - m_prevCpuUser;
            const quint64 totalDiff = kernelDiff + userDiff;
            if (totalDiff > 0) {
                const double usage = std::clamp(
                    (static_cast<double>(totalDiff - idleDiff) * 100.0) / static_cast<double>(totalDiff),
                    0.0, 100.0);
                m_currentCpuUsage = usage;
                appendUsageHistory(m_cpuUsageHistory, usage);
                cpuText = QString("CPU  %1%").arg(usage, 0, 'f', 0);
                applyUsageColor(m_cpuUsageItem, usage);
                applyMetricLevel(m_systemCpuValueLabel, usage, QString("%1%").arg(usage, 0, 'f', 0));
            }
        }

        m_prevCpuIdle = idle;
        m_prevCpuKernel = kernel;
        m_prevCpuUser = user;
        m_hasCpuSample = true;
        m_cpuUsageItem->setText(0, cpuText);
    }

    MEMORYSTATUSEX mem{};
    mem.dwLength = sizeof(mem);
    if (GlobalMemoryStatusEx(&mem)) {
        const double usedGb =
            static_cast<double>(mem.ullTotalPhys - mem.ullAvailPhys) / (1024.0 * 1024.0 * 1024.0);
        const double totalGb =
            static_cast<double>(mem.ullTotalPhys) / (1024.0 * 1024.0 * 1024.0);
        m_memoryUsageItem->setText(
            0,
            QString("Memory  %1% (%2 / %3 GB)")
                .arg(mem.dwMemoryLoad)
                .arg(usedGb, 0, 'f', 1)
                .arg(totalGb, 0, 'f', 1));
        m_currentMemoryUsage = static_cast<double>(mem.dwMemoryLoad);
        appendUsageHistory(m_memoryUsageHistory, m_currentMemoryUsage);
        applyUsageColor(m_memoryUsageItem, m_currentMemoryUsage);
        applyMetricLevel(m_systemMemoryValueLabel, m_currentMemoryUsage, QString("%1%").arg(mem.dwMemoryLoad));
    }
#else
    m_cpuUsageItem->setText(0, "CPU  -");
    m_memoryUsageItem->setText(0, "Memory  -");
    applyMetricLevel(m_systemCpuValueLabel, 0.0, "-");
    applyMetricLevel(m_systemMemoryValueLabel, 0.0, "-");
#endif

    requestGpuUsageUpdate();
    if (m_systemUsageWindow) {
        m_systemUsageWindow->setUsageData(
            m_currentCpuUsage,
            m_currentMemoryUsage,
            m_currentGpuUsage,
            m_cpuUsageHistory,
            m_memoryUsageHistory,
            m_gpuUsageHistory);
    }
}

void MainWindow::requestGpuUsageUpdate()
{
#ifndef Q_OS_WIN
    if (m_gpuUsageItem) m_gpuUsageItem->setText(0, "GPU  -");
    if (m_systemGpuValueLabel) m_systemGpuValueLabel->setText("-");
    return;
#else
    if (!m_gpuUsageItem) return;
    if (m_gpuUsageProcess && m_gpuUsageProcess->state() != QProcess::NotRunning) return;
    if (!m_gpuUsageProcess) {
        m_gpuUsageProcess = new QProcess(this);
        connect(m_gpuUsageProcess,
                qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
                this,
                [this](int, QProcess::ExitStatus) {
            if (!m_gpuUsageItem || !m_gpuUsageProcess) return;
            const QString output = QString::fromLocal8Bit(m_gpuUsageProcess->readAllStandardOutput()).trimmed();
            bool ok = false;
            const double value = output.toDouble(&ok);
            m_currentGpuUsage = ok ? value : -1.0;
            if (ok) appendUsageHistory(m_gpuUsageHistory, value);
            m_gpuUsageItem->setText(0, ok ? QString("GPU  %1%").arg(value, 0, 'f', 0) : "GPU  -");
            if (ok) {
                QColor color("#dce3ff");
                if (value >= 85.0) color = QColor("#ff8d8d");
                else if (value >= 65.0) color = QColor("#f3c86b");
                m_gpuUsageItem->setForeground(0, QBrush(color));
            }
            if (m_systemGpuValueLabel) {
                m_systemGpuValueLabel->setText(ok ? QString("%1%").arg(value, 0, 'f', 0) : "-");
                QString level;
                if (ok && value >= 85.0) level = "danger";
                else if (ok && value >= 65.0) level = "warn";
                m_systemGpuValueLabel->setProperty("level", level);
                m_systemGpuValueLabel->style()->unpolish(m_systemGpuValueLabel);
                m_systemGpuValueLabel->style()->polish(m_systemGpuValueLabel);
            }
            if (m_systemUsageWindow) {
                m_systemUsageWindow->setUsageData(
                    m_currentCpuUsage,
                    m_currentMemoryUsage,
                    m_currentGpuUsage,
                    m_cpuUsageHistory,
                    m_memoryUsageHistory,
                    m_gpuUsageHistory);
            }
        });
    }

    const QString script =
        "$v=(Get-Counter '\\GPU Engine(*)\\Utilization Percentage' -ErrorAction SilentlyContinue).CounterSamples | "
        "Measure-Object -Property CookedValue -Sum; "
        "if($v -and $v.Sum -ne $null){ [Math]::Round([Math]::Min([double]$v.Sum,100),0) }";
    m_gpuUsageProcess->start(
        "powershell",
        {"-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", script});
#endif
}

// ---------------- central live (GStreamer) ----------------
void MainWindow::switchCentralChannel(int ch)
{
    m_centralCh = ch;
    syncCentralOverlayGeometry();

    const QString url = m_centralRtsp.value(ch).trimmed();
    if (url.isEmpty()) {
        appendLog(QString("Central CH%1 url empty").arg(ch));
        return;
    }

    if (!ui->labelCentralView) {
        appendLog("labelCentralView widget not found (check Promote/objectName)");
        return;
    }

    if (m_centralPlayingUrl == url) {
        appendLog(QString("Central CH%1 same url -> skip").arg(ch));
        return;
    }

    if (m_primaryViewMode != PrimaryViewMode::Tank) {
        ui->labelCentralView->stopStream();
        ui->labelCentralView->startStream(url);
        updateCentralLayerOrder();
    }

    m_centralPlayingUrl = url;
    appendLog(QString("Central switch -> CH%1").arg(ch));
}

// ---------------- tree click routing ----------------
void MainWindow::onWardItemClicked(QTreeWidgetItem* item, int column)
{
    Q_UNUSED(column);
    if (!item) return;
    if (item == m_systemUsageRoot ||
        item == m_cpuUsageItem ||
        item == m_memoryUsageItem ||
        item == m_gpuUsageItem) {
        if (!m_systemUsageWindow) {
            m_systemUsageWindow = new SystemUsageWindow(this);
            m_systemUsageWindow->setAttribute(Qt::WA_DeleteOnClose, false);
        }
        m_systemUsageWindow->setUsageData(
            m_currentCpuUsage,
            m_currentMemoryUsage,
            m_currentGpuUsage,
            m_cpuUsageHistory,
            m_memoryUsageHistory,
            m_gpuUsageHistory);
        m_systemUsageWindow->show();
        m_systemUsageWindow->raise();
        m_systemUsageWindow->activateWindow();
        return;
    }

    // 洹몃９?占쎈㈃ ?占쏙옙?占?
    if (item->childCount() > 0) {
        item->setExpanded(!item->isExpanded());
        return;
    }

    const QString key = item->text(0).trimmed();

    // ???占쏙옙?/六μ뭅占??占쎈Т 寃껊룄 ????
    if (!m_wardStreams.contains(key)) {
        appendLog(QString("Dummy item (no stream): %1").arg(key));
        return;
    }

    // ????prefix占?占??占쎌슦??
    if (key.startsWith("Main")) {
        switchCentralStreamTo(key);
    } else if (key.startsWith("T")) {
        switchTankStreamTo(key);
    } else {
        appendLog(QString("Unknown key type: %1").arg(key));
    }
}

void MainWindow::switchCentralStreamTo(const QString& key)
{
    if (!ui->labelCentralView) {
        appendLog("labelCentralView not found");
        return;
    }

    const auto p = m_wardStreams.value(key);
    const QString url = p.rawRtsp.trimmed(); // raw占??占쎌슜

    if (url.isEmpty()) return;
    if (m_centralPlayingUrl == url) {
        appendLog(QString("Central same url -> skip (%1)").arg(key));
        return;
    }

    if (m_primaryViewMode != PrimaryViewMode::Tank) {
        ui->labelCentralView->stopStream();
        ui->labelCentralView->startStream(url);
    }
    m_centralPlayingUrl = url;

    appendLog(QString("Central switch -> %1").arg(key));
}

void MainWindow::switchTankStreamTo(const QString& key)
{
    if (!ui->labelTankView) {
        appendLog("labelTankView not found (Promote/objectName ?占쎌씤)");
        return;
    }

    const auto p = m_wardStreams.value(key);
    const QString url = p.rawRtsp.trimmed();
    appendLog(QString("switchTankStreamTo(%1) -> %2").arg(key, url.isEmpty() ? "<empty>" : url));

    if (url.isEmpty()) return;
    if (m_tankPlayingUrl == url) {
        appendLog(QString("Tank same url -> skip (%1)").arg(key));
        return;
    }

    if (m_primaryViewMode == PrimaryViewMode::Tank) {
        appendLog(QString("Tank immediate startStream -> %1").arg(url));
        ui->labelTankView->stopStream();
        ui->labelTankView->startStream(url);
    }
    m_tankPlayingUrl = url;

    appendLog(QString("Tank switch -> %1").arg(key));
}

void MainWindow::switchPatrolStreamTo(const QString& key)
{
    if (!ui->labelPatrolView) {
        appendLog("labelPatrolView not found (Promote/objectName ?占쎌씤)");
        return;
    }

    const auto p = m_wardStreams.value(key);
    const QString url = p.rawRtsp.trimmed();

    if (url.isEmpty()) return;
    if (m_patrolPlayingUrl == url) {
        appendLog(QString("Patrol same url -> skip (%1)").arg(key));
        return;
    }

    ui->labelPatrolView->stopStream();
    ui->labelPatrolView->startStream(url);
    m_patrolPlayingUrl = url;

    appendLog(QString("Patrol switch -> %1").arg(key));
}

// ---------------- event ----------------
void MainWindow::onEventClicked(QListWidgetItem* item)
{
    if (!item) return;

    const int idx = item->data(Qt::UserRole).toInt();
    if (idx < 0 || idx >= m_events.size()) {
        appendLog("Invalid event index");
        return;
    }

    const EventItem& ev = m_events[idx];

    // ===== MQTT ?占쎈깽???占쎈┃ ?占쎌깮 =====
    if (ev.source == "mqtt") {
        return;
    }

    // ===== 湲곗〈 ward ?占쎈깽??泥섎━ ?占쏙옙? =====
    if (m_role == UserRole::ControlRoom && ev.source == "ward" && ev.privacyClip.isEmpty()) {
        appendLog("ControlRoom: ward privacy clip missing -> cannot play");
        return;
    }

    if (!m_clipWin)
        m_clipWin = new VideoClipWindow(this);

    VideoClipWindow::UserRole clipRole =
        (m_role == UserRole::Executive) ? VideoClipWindow::UserRole::Executive
                                        : VideoClipWindow::UserRole::ControlRoom;

    m_clipWin->openClip(
        clipTitle(ev),
        clipRole,
        QUrl(ev.privacyClip),
        QUrl(ev.rawClip)
        );

}

void MainWindow::onEventDoubleClicked(QListWidgetItem* item)
{
    if (!item) return;
    if (tryOpenMqttDangerClip(item)) return;
    onEventClicked(item);
}

bool MainWindow::tryOpenMqttDangerClip(QListWidgetItem* item)
{
    if (!item) return false;

    const int idx = item->data(Qt::UserRole).toInt();
    if (idx < 0 || idx >= m_events.size()) return false;

    const EventItem& ev = m_events[idx];
    if (ev.source != "mqtt") return false;

    const QString clipUrl  = item->data(Qt::UserRole + 1).toString();
    int clipSec            = item->data(Qt::UserRole + 2).toInt();
    const QString topic    = item->data(Qt::UserRole + 3).toString();
    const QString cam      = item->data(Qt::UserRole + 4).toString();
    const QString utcShort = item->data(Qt::UserRole + 5).toString();

    if (!isDangerEventTopic(topic)) {
        return true;
    }

    if (clipUrl.isEmpty()) {
        appendLog("MQTT danger event but clip_url is empty");
        return true;
    }

    if (clipSec <= 0) clipSec = 5;

    if (!m_clipPopup) {
        m_clipPopup = new ClipPopup(this);
        m_clipPopup->setAttribute(Qt::WA_DeleteOnClose, false);
    }

    m_clipPopup->show();
    m_clipPopup->raise();
    m_clipPopup->activateWindow();
    m_clipPopup->playClip(
        cam,
        topic,
        utcShort,
        QUrl(clipUrl),
        clipSec
    );

    return true;
}

// ---------------- all cctv ----------------
void MainWindow::onOpenAllCctv()
{
    if (!m_allCctvWin) {
        m_allCctvWin = new AllCctvWindow(nullptr);

        m_allCctvWin->setAttribute(Qt::WA_DeleteOnClose, true);
        connect(m_allCctvWin, &QObject::destroyed, this, [this]() {
            m_allCctvWin = nullptr;
        });
    }

    // AllCCTV???占쏀듃占?占?二쇱엯
    QMap<int, QString> rtsp;
    rtsp[1] = m_centralRtsp.value(1);
    rtsp[2] = !m_tankPlayingUrl.trimmed().isEmpty()
        ? m_tankPlayingUrl.trimmed()
        : m_wardStreams.value("T1").rawRtsp.trimmed();
    rtsp[3] = m_centralRtsp.value(1);
    rtsp[4] = m_centralRtsp.value(1);
    rtsp[5] = m_centralRtsp.value(1);
    rtsp[6] = m_centralRtsp.value(1);
    rtsp[7] = m_centralRtsp.value(1);

    QMap<int, QString> titles;
    titles[1] = "Center CH1";
    titles[2] = "Tank CH1";
    titles[3] = "Center CH1 Gray";
    titles[4] = "Center CH1 Edge";
    titles[5] = "Center CH1 Inverted";
    titles[6] = "Center CH1 Zoom";
    titles[7] = "Center CH1 Motion";

    m_allCctvWin->setStreams(rtsp, titles);

    m_allCctvWin->show();
    m_allCctvWin->raise();
    m_allCctvWin->activateWindow();
}

// ---------------- utils ----------------
void MainWindow::appendLog(const QString& line)
{
    const QString ts = QDateTime::currentDateTime().toString("hh:mm:ss");
    ui->textMsgLog->appendPlainText(QString("[%1] %2").arg(ts, line));
    if (ui && ui->textMsgLog) {
        if (auto* bar = ui->textMsgLog->verticalScrollBar()) {
            bar->setValue(bar->maximum());
        }
        ui->textMsgLog->ensureCursorVisible();
        QMetaObject::invokeMethod(ui->textMsgLog, [log = ui->textMsgLog] {
            if (!log) return;
            if (auto* bar = log->verticalScrollBar()) {
                bar->setValue(bar->maximum());
            }
            log->ensureCursorVisible();
        }, Qt::QueuedConnection);
    }
}

QString MainWindow::clipTitle(const EventItem& ev) const
{
    const QString roleText = (m_role == UserRole::Executive) ? "Executive" : "ControlRoom";
    const QString src = ev.source;
    const QString ward = ev.wardName.isEmpty() ? "" : QString(" | %1").arg(ev.wardName);
    return QString("Clip | %1 | %2%3 | %4").arg(roleText, src, ward, ev.id);
}

void MainWindow::setupRuViewInfoUi()
{
    auto* msgLayout = findChild<QVBoxLayout*>("verticalLayout_msg");
    if (!msgLayout) return;

    auto* card = new QFrame(this);
    card->setObjectName("ruviewInfoCard");
    card->setFrameShape(QFrame::StyledPanel);
    auto* cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(10, 8, 10, 8);
    cardLayout->setSpacing(4);

    auto* eyebrow = new QLabel("RUVIEW SUMMARY", card);
    eyebrow->setProperty("eventSectionEyebrow", true);
    m_ruviewStatusLabel = new QLabel("RuView Status: UI OFF", card);
    m_ruviewStateLabel = new QLabel("State: -", card);
    m_ruviewConfLabel = new QLabel("Confidence: -", card);
    m_ruviewNodesLabel = new QLabel("Detected/Active Nodes: -", card);
    m_ruviewStatusLabel->setProperty("ruCardTitle", true);
    m_ruviewStateLabel->setProperty("ruCardValue", true);
    m_ruviewConfLabel->setProperty("ruCardValue", true);
    m_ruviewNodesLabel->setProperty("ruCardValue", true);

    cardLayout->addWidget(eyebrow);
    cardLayout->addWidget(m_ruviewStatusLabel);
    cardLayout->addWidget(m_ruviewStateLabel);
    cardLayout->addWidget(m_ruviewConfLabel);
    cardLayout->addWidget(m_ruviewNodesLabel);

    msgLayout->insertWidget(0, card);
}

void MainWindow::setupEventFilterUi()
{
    auto* msgLayout = findChild<QVBoxLayout*>("verticalLayout_msg");
    if (!msgLayout) return;

    auto* headerRow = new QWidget(this);
    headerRow->setObjectName("eventHeaderRow");
    auto* headerLayout = new QHBoxLayout(headerRow);
    headerLayout->setContentsMargins(0, 0, 0, 0);
    headerLayout->setSpacing(8);
    auto* headerTextCol = new QVBoxLayout();
    headerTextCol->setContentsMargins(0, 0, 0, 0);
    headerTextCol->setSpacing(1);
    auto* headerEyebrow = new QLabel("LIVE FEED", headerRow);
    headerEyebrow->setProperty("eventSectionEyebrow", true);
    auto* headerTitle = new QLabel("Event Queue", headerRow);
    headerTitle->setProperty("eventSectionTitle", true);
    headerTextCol->addWidget(headerEyebrow);
    headerTextCol->addWidget(headerTitle);
    m_eventCountLabel = new QLabel("0", headerRow);
    m_eventCountLabel->setObjectName("eventCountBadge");
    headerLayout->addLayout(headerTextCol, 1);
    headerLayout->addWidget(m_eventCountLabel, 0, Qt::AlignCenter);
    msgLayout->insertWidget(1, headerRow);

    auto* filterPanel = new QWidget(this);
    filterPanel->setObjectName("eventFilterPanel");
    auto* filterPanelLayout = new QVBoxLayout(filterPanel);
    filterPanelLayout->setContentsMargins(10, 10, 10, 10);
    filterPanelLayout->setSpacing(8);

    auto* searchRow = new QWidget(filterPanel);
    auto* searchLayout = new QHBoxLayout(searchRow);
    searchLayout->setContentsMargins(0, 0, 0, 0);
    searchLayout->setSpacing(6);
    auto* searchLabel = new QLabel("Search", searchRow);
    searchLabel->setObjectName("eventFilterLabel");
    m_eventSearchEdit = new QLineEdit(searchRow);
    m_eventSearchEdit->setObjectName("eventFilterSearch");
    m_eventSearchEdit->setPlaceholderText("Find events...");
    connect(m_eventSearchEdit, &QLineEdit::textChanged, this, [this](const QString&) {
        refreshEventFilter();
    });
    searchLayout->addWidget(searchLabel);
    searchLayout->addWidget(m_eventSearchEdit, 1);
    filterPanelLayout->addWidget(searchRow);

    auto* row = new QWidget(filterPanel);
    auto* hl = new QHBoxLayout(row);
    hl->setContentsMargins(0, 0, 0, 0);
    hl->setSpacing(6);

    auto* label = new QLabel("Event Filter", row);
    label->setObjectName("eventFilterLabel");
    m_eventFilter = new QComboBox(row);
    m_eventFilter->setObjectName("eventFilterCombo");
    m_eventFilter->addItem("All");
    m_eventFilter->addItem("CCTV area");
    m_eventFilter->addItem("CCTV human");
    m_eventFilter->addItem("RuView");
    m_eventFilter->setCurrentIndex(0);
    connect(m_eventFilter, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int){ refreshEventFilter(); });

    hl->addWidget(label);
    hl->addWidget(m_eventFilter, 1);
    filterPanelLayout->addWidget(row);

    auto* controlsRow = new QWidget(filterPanel);
    auto* controlsLayout = new QHBoxLayout(controlsRow);
    controlsLayout->setContentsMargins(0, 0, 0, 0);
    controlsLayout->setSpacing(6);
    m_eventSortButton = new QPushButton("Newest", controlsRow);
    m_eventSortButton->setCheckable(true);
    m_eventSortButton->setChecked(true);
    m_eventSortButton->setProperty("eventControl", "toggle");
    m_eventPauseButton = new QPushButton("Pause", controlsRow);
    m_eventPauseButton->setCheckable(true);
    m_eventPauseButton->setProperty("eventControl", "secondary");
    m_eventClearButton = new QPushButton("Clear", controlsRow);
    m_eventClearButton->setProperty("eventControl", "danger");
    m_autoClipPopupButton = new QPushButton("Auto Clip", controlsRow);
    m_autoClipPopupButton->setCheckable(true);
    m_autoClipPopupButton->setChecked(true);
    m_autoClipPopupButton->setProperty("eventControl", "toggle");
    connect(m_eventPauseButton, &QPushButton::toggled, this, [this](bool on) {
        m_eventListPaused = on;
        if (m_eventPauseButton) m_eventPauseButton->setText(on ? "Paused" : "Pause");
    });
    connect(m_eventSortButton, &QPushButton::toggled, this, [this](bool on) {
        m_eventAutoSortEnabled = on;
        if (m_eventSortButton) m_eventSortButton->setText(on ? "Newest" : "Append");
        saveUiSettings();
    });
    connect(m_eventClearButton, &QPushButton::clicked, this, [this]() {
        if (ui && ui->listEvent) ui->listEvent->clear();
        m_events.clear();
        updateEventCountBadge();
        appendLog("Event list cleared");
    });
    connect(m_autoClipPopupButton, &QPushButton::toggled, this, [this](bool on) {
        m_autoClipPopupEnabled = on;
        saveUiSettings();
    });
    controlsLayout->addWidget(m_eventSortButton);
    controlsLayout->addWidget(m_eventPauseButton);
    controlsLayout->addWidget(m_eventClearButton);
    controlsLayout->addWidget(m_autoClipPopupButton);
    controlsLayout->addStretch();
    filterPanelLayout->addWidget(controlsRow);
    msgLayout->insertWidget(2, filterPanel);

    if (ui && ui->textMsgLog) {
        auto* logToggle = new QToolButton(this);
        logToggle->setObjectName("logToggleButton");
        logToggle->setText("Show event log");
        logToggle->setCheckable(true);
        logToggle->setChecked(false);

        auto* logContainer = new QWidget(this);
        logContainer->setObjectName("eventLogContainer");
        logContainer->setVisible(false);
        auto* logLayout = new QVBoxLayout(logContainer);
        logLayout->setContentsMargins(0, 0, 0, 0);
        logLayout->setSpacing(6);
        ui->textMsgLog->setVisible(false);
        logLayout->addWidget(ui->textMsgLog);

        connect(logToggle, &QToolButton::toggled, this, [this, logToggle, logContainer](bool on) {
            logToggle->setText(on ? "Hide event log" : "Show event log");
            logContainer->setVisible(on);
            if (ui && ui->textMsgLog) {
                ui->textMsgLog->setVisible(on);
            }
        });

        msgLayout->addWidget(logToggle, 0, Qt::AlignRight);
        msgLayout->addWidget(logContainer, 0);
    }

    connect(m_eventFilter, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { saveUiSettings(); });
}

void MainWindow::loadUiSettings()
{
    QSettings s("VEDA", "SentinelFusion");

    m_mapNavExpanded = s.value("main/map_nav_expanded", true).toBool();
    m_eventAutoSortEnabled = s.value("main/event_auto_sort", true).toBool();
    m_autoClipPopupEnabled = s.value("main/autoclip_enabled", true).toBool();

    if (m_eventSortButton) {
        const QSignalBlocker blocker(m_eventSortButton);
        m_eventSortButton->setChecked(m_eventAutoSortEnabled);
        m_eventSortButton->setText(m_eventAutoSortEnabled ? "Newest" : "Append");
    }

    if (m_autoClipPopupButton) {
        const QSignalBlocker blocker(m_autoClipPopupButton);
        m_autoClipPopupButton->setChecked(m_autoClipPopupEnabled);
    }

    if (m_eventFilter) {
        const QSignalBlocker blocker(m_eventFilter);
        m_eventFilter->setCurrentIndex(s.value("main/event_filter_index", 0).toInt());
    }

    const bool mapEnabled = s.value("main/map_overlay_enabled", false).toBool();
    const bool detectionEnabled = s.value("main/detection_enabled", false).toBool();
    const bool ruviewEnabled = s.value("main/ruview_enabled", false).toBool();

    if (mapEnabled != m_mapOverlayEnabled) {
        if (auto* b = findChild<QPushButton*>("btnMap")) b->click();
    }
    if (detectionEnabled != m_humanBoxEnabled) {
        if (auto* b = findChild<QPushButton*>("Humanbtn")) b->click();
    }
    if (ruviewEnabled != m_ruviewUiEnabled) {
        if (auto* b = findChild<QPushButton*>("ruviewbtn")) b->click();
    }

    rebuildPrimaryNavList();
    refreshEventFilter();
}

void MainWindow::saveUiSettings() const
{
    QSettings s("VEDA", "SentinelFusion");
    s.setValue("main/detection_enabled", m_humanBoxEnabled);
    s.setValue("main/ruview_enabled", m_ruviewUiEnabled);
    s.setValue("main/autoclip_enabled", m_autoClipPopupEnabled);
    s.setValue("main/event_filter_index", m_eventFilter ? m_eventFilter->currentIndex() : 0);
    s.setValue("main/map_overlay_enabled", m_mapOverlayEnabled);
    s.setValue("main/map_nav_expanded", m_mapNavExpanded);
    s.setValue("main/event_auto_sort", m_eventAutoSortEnabled);
}

QString MainWindow::buildEventDedupKey(const MqttEvent& ev, const QString& srcLabel, const QString& locationLabel) const
{
    return QString("%1|%2|%3|%4|%5|%6|%7|%8|%9")
        .arg(srcLabel, locationLabel, ev.topic, ev.rule, ev.action, ev.level, ev.objectType, ev.sensorId, ev.cam);
}

QString MainWindow::formatEventItemText(const QString& firstLine, const QString& secondLine, int duplicateCount) const
{
    QString second = secondLine;
    if (duplicateCount > 1) {
        second += QString(" x%1").arg(duplicateCount);
    }
    return firstLine + "\n " + second;
}

QWidget* MainWindow::buildEventItemWidget(QListWidgetItem* item) const
{
    if (!item || !ui || !ui->listEvent) return nullptr;

    const QString firstLine = item->data(Qt::UserRole + 12).toString();
    const QString secondLine = item->data(Qt::UserRole + 13).toString();
    const int duplicateCount = std::max(1, item->data(Qt::UserRole + 14).toInt());
    const QString topic = item->data(Qt::UserRole + 3).toString();
    const QString src = item->data(Qt::UserRole + 6).toString();
    const QString utcShort = item->data(Qt::UserRole + 5).toString();
    const bool hasClip = !item->data(Qt::UserRole + 1).toString().isEmpty();

    QString severity = "normal";
    const QColor foreground = item->foreground().color();
    if (foreground.red() >= 220 && foreground.green() <= 130) severity = "danger";
    else if (foreground.red() >= 210 && foreground.green() >= 120) severity = "warn";

    auto* card = new QFrame(ui->listEvent);
    card->setProperty("eventItemCard", true);
    card->setProperty("severity", severity);

    auto* root = new QHBoxLayout(card);
    root->setContentsMargins(10, 8, 10, 8);
    root->setSpacing(10);

    auto* accent = new QFrame(card);
    accent->setProperty("eventAccent", true);
    accent->setStyleSheet(QString("background-color:%1;").arg(foreground.name()));
    root->addWidget(accent);

    auto* content = new QVBoxLayout();
    content->setContentsMargins(0, 0, 0, 0);
    content->setSpacing(4);

    auto* topRow = new QHBoxLayout();
    topRow->setContentsMargins(0, 0, 0, 0);
    topRow->setSpacing(6);
    auto* metaLabel = new QLabel(QString("%1  %2").arg(utcShort, src.toUpper()), card);
    metaLabel->setProperty("eventMeta", true);
    auto* badgeLabel = new QLabel(hasClip ? "CLIP" : topic.toUpper(), card);
    badgeLabel->setProperty("eventBadge", true);
    badgeLabel->setProperty("kind", severity);
    topRow->addWidget(metaLabel, 0, Qt::AlignVCenter);
    topRow->addStretch();
    topRow->addWidget(badgeLabel, 0, Qt::AlignVCenter);

    auto* titleLabel = new QLabel(firstLine, card);
    titleLabel->setProperty("eventTitle", true);
    titleLabel->setWordWrap(true);
    auto* subtitleLabel = new QLabel(duplicateCount > 1
                                         ? QString("%1  x%2").arg(secondLine).arg(duplicateCount)
                                         : secondLine,
                                     card);
    subtitleLabel->setProperty("eventSubtitle", true);
    subtitleLabel->setWordWrap(true);

    content->addLayout(topRow);
    content->addWidget(titleLabel);
    content->addWidget(subtitleLabel);
    root->addLayout(content, 1);

    return card;
}

void MainWindow::refreshEventItemWidget(QListWidgetItem* item)
{
    if (!item || !ui || !ui->listEvent) return;
    if (QWidget* existing = ui->listEvent->itemWidget(item)) {
        existing->deleteLater();
    }
    if (QWidget* card = buildEventItemWidget(item)) {
        card->ensurePolished();
        card->adjustSize();
        const QSize hint = card->sizeHint().expandedTo(QSize(0, 88));
        item->setSizeHint(hint);
        ui->listEvent->setItemWidget(item, card);
    } else {
        item->setSizeHint(QSize(0, 88));
    }
}

bool MainWindow::tryMergeDuplicateEvent(const QString& dedupKey,
                                        const QString& firstLine,
                                        const QString& secondLine,
                                        const MqttEvent& ev,
                                        const QString& src,
                                        const QString& utcShort,
                                        const QColor& color)
{
    if (!ui || !ui->listEvent) return false;

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    for (int i = 0; i < ui->listEvent->count(); ++i) {
        auto* item = ui->listEvent->item(i);
        if (!item) continue;
        if (item->data(Qt::UserRole + 10).toString() != dedupKey) continue;

        const qint64 lastMs = item->data(Qt::UserRole + 11).toLongLong();
        if (nowMs - lastMs > 3000) continue;

        const int nextCount = std::max(1, item->data(Qt::UserRole + 14).toInt()) + 1;
        item->setData(Qt::UserRole + 1, ev.clipUrl);
        item->setData(Qt::UserRole + 2, ev.clipSec > 0 ? ev.clipSec : 5);
        item->setData(Qt::UserRole + 3, ev.topic);
        item->setData(Qt::UserRole + 4, ev.cam);
        item->setData(Qt::UserRole + 5, utcShort);
        item->setData(Qt::UserRole + 6, src);
        item->setData(Qt::UserRole + 7, ev.level);
        item->setData(Qt::UserRole + 11, nowMs);
        item->setData(Qt::UserRole + 12, firstLine);
        item->setData(Qt::UserRole + 13, secondLine);
        item->setData(Qt::UserRole + 14, nextCount);
        item->setText(formatEventItemText(firstLine, secondLine, nextCount));
        item->setForeground(QBrush(color));
        refreshEventItemWidget(item);
        item->setHidden(!shouldShowEventItem(item));

        if (m_eventAutoSortEnabled && i > 0) {
            auto* moved = ui->listEvent->takeItem(i);
            ui->listEvent->insertItem(0, moved);
            refreshEventItemWidget(moved);
        }
        return true;
    }

    return false;
}

bool MainWindow::shouldShowEventItem(const QListWidgetItem* item) const
{
    if (!item || !m_eventFilter) return true;

    const int mode = m_eventFilter->currentIndex();
    const QString src = item->data(Qt::UserRole + 6).toString();
    const QString level = item->data(Qt::UserRole + 7).toString().toLower();
    const QString topic = item->data(Qt::UserRole + 3).toString();
    const bool isRuview = src.compare("ruview", Qt::CaseInsensitive) == 0;
    const bool isCctv = !isRuview;
    const QString keyword = m_eventSearchEdit ? m_eventSearchEdit->text().trimmed() : QString();

    if (!keyword.isEmpty()) {
        const QString haystack = QString("%1 %2 %3")
                                     .arg(item->text())
                                     .arg(topic)
                                     .arg(src);
        if (!haystack.contains(keyword, Qt::CaseInsensitive)) return false;
    }

    if (mode == 0) return true; // All
    if (mode == 1) return isCctv && topic.compare("IvaArea", Qt::CaseInsensitive) == 0;
    if (mode == 2) return isCctv && topic.compare("ObjectDetection", Qt::CaseInsensitive) == 0;
    if (mode == 3) return isRuview && (level == "fused" || level == "node" || level == "pose" || level == "coarse");
    return true;
}

void MainWindow::refreshEventFilter()
{
    if (!ui || !ui->listEvent) return;
    int visibleCount = 0;
    for (int i = 0; i < ui->listEvent->count(); ++i) {
        auto* item = ui->listEvent->item(i);
        if (!item) continue;
        const bool show = shouldShowEventItem(item);
        item->setHidden(!show);
        if (show) ++visibleCount;
    }
    if (m_eventCountLabel) m_eventCountLabel->setText(QString::number(visibleCount));
}

void MainWindow::updateEventCountBadge()
{
    refreshEventFilter();
}

void MainWindow::clearRuViewUiState()
{
    m_ruviewOnline = false;
    m_ruviewPresent = false;
    m_ruviewConfidence = 0.0;
    m_ruviewActiveNodes = 0;
    m_ruviewDetectedNodes = 0;
    applyRuViewZoneVisual(false, 0.0);
    if (m_poseOverlay) m_poseOverlay->setPoseFrame(PoseFrame{});
    if (m_clipPopup && m_clipPopup->isVisible()) {
        m_clipPopup->hide();
    }
}

void MainWindow::updateCentralAspectRatio()
{
    if (!ui || !ui->centralOverlayHost) return;
    auto* host = findChild<QWidget*>("centralAspectHost");
    if (!host) return;

    const int availW = host->width();
    const int availH = host->height();
    if (availW <= 0 || availH <= 0) return;
    if (availW < 280 || availH < 180) return;

    int targetW = availW;
    int targetH = static_cast<int>(std::round(targetW / m_centralAspect));
    if (targetH > availH) {
        targetH = availH;
        targetW = static_cast<int>(std::round(targetH * m_centralAspect));
    }
    targetW = std::max(1, targetW);
    targetH = std::max(1, targetH);

    ui->centralOverlayHost->setMinimumSize(targetW, targetH);
    ui->centralOverlayHost->setMaximumSize(targetW, targetH);
    ui->centralOverlayHost->resize(targetW, targetH);
    ui->centralOverlayHost->updateGeometry();

}

void MainWindow::updateSideVideoAspectRatios()
{
    const auto applyAspect = [this](const char* hostName, QWidget* target) {
        if (!target) return;
        auto* host = findChild<QWidget*>(hostName);
        if (!host) return;
        const int availW = host->width();
        const int availH = host->height();
        if (availW <= 0 || availH <= 0) return;
        if (availW < 180 || availH < 120) return;

        // Tank view should span the full available width and use a shorter height.
        const double tankAspect = 16.0 / 9.0;
        int targetW = availW;
        int targetH = static_cast<int>(std::round(targetW / tankAspect));
        if (targetH > availH) {
            targetH = availH;
        }
        targetW = std::max(1, targetW);
        targetH = std::max(1, targetH);
        target->setMinimumSize(targetW, targetH);
        target->setMaximumSize(targetW, targetH);
        target->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        target->show();
        target->resize(targetW, targetH);
        target->updateGeometry();
    };

    applyAspect("tankAspectHost", ui ? ui->labelTankView : nullptr);
}

void MainWindow::applyMainSplitterDefaultSizes()
{
    if (!m_mainSplitter) return;
    if (m_mainSplitter->count() < 3) return;
    const int total = m_mainSplitter->width();
    if (total <= 0) return;

    int nav = static_cast<int>(std::round(total * 0.145));
    int right = static_cast<int>(std::round(total * 0.21));
    int middle = total - nav - right;

    auto clampByWidget = [](int value, QWidget* w) {
        if (!w) return value;
        int v = std::max(value, w->minimumWidth());
        const int maxW = w->maximumWidth();
        if (maxW > 0 && maxW < QWIDGETSIZE_MAX) v = std::min(v, maxW);
        return v;
    };

    QWidget* w0 = m_mainSplitter->widget(0);
    QWidget* w1 = m_mainSplitter->widget(1);
    QWidget* w2 = m_mainSplitter->widget(2);
    nav = clampByWidget(nav, w0);
    middle = clampByWidget(middle, w1);
    right = clampByWidget(right, w2);

    int sum = nav + middle + right;
    if (sum != total && total > 0) {
        const int diff = total - sum;
        middle = std::max((w1 ? w1->minimumWidth() : 1), middle + diff);
    }

    m_mainSplitter->setSizes({nav, middle, right});
}

bool MainWindow::isRuViewEvent(const MqttEvent& ev) const
{
    if (ev.messageType.compare(kVisionPoseBridgeStatusType, Qt::CaseInsensitive) == 0) return true;
    if (ev.messageType.compare("rf_person_presence", Qt::CaseInsensitive) == 0) return true;
    if (ev.messageType.compare("vision_pose", Qt::CaseInsensitive) == 0) return true;
    if (ev.messageType.compare("coarse_pose", Qt::CaseInsensitive) == 0) return true;
    if (ev.src.compare("ruview", Qt::CaseInsensitive) == 0) return true;
    if (ev.src.compare("ruview_bridge", Qt::CaseInsensitive) == 0) return true;
    if (ev.topicFull.contains("/ruview/", Qt::CaseInsensitive)) return true;
    if (ev.topicFull.contains("/person_view", Qt::CaseInsensitive)) return true;
    if ((ev.topic.compare("presence", Qt::CaseInsensitive) == 0 ||
         ev.topic.compare("rf_presence", Qt::CaseInsensitive) == 0) &&
        !ev.level.isEmpty()) return true;
    return false;
}

void MainWindow::loadRuViewZoneConfig()
{
    m_ruviewNodeZoneMap.clear();
    m_ruviewAlertZones.clear();

    QFile file(QString::fromUtf8(kRuViewZoneConfigPath));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        appendLog(QString("RuView zone config load failed: %1").arg(file.fileName()));
        return;
    }

    const QString text = QString::fromUtf8(file.readAll());
    file.close();

    const QRegularExpression mapRe(QStringLiteral("export\\s+SENSOR_ZONE_MAP='([^']+)'"));
    const QRegularExpression zone1Re(QStringLiteral("ZONE1_NODES=\"\\s*([^\"]*)\""));
    const QRegularExpression zone2Re(QStringLiteral("ZONE2_NODES=\"\\s*([^\"]*)\""));

    const QRegularExpressionMatch mapMatch = mapRe.match(text);
    if (mapMatch.hasMatch()) {
        QJsonParseError err{};
        const QJsonDocument doc = QJsonDocument::fromJson(mapMatch.captured(1).toUtf8(), &err);
        if (err.error == QJsonParseError::NoError && doc.isObject()) {
            const QJsonObject obj = doc.object();
            for (auto it = obj.begin(); it != obj.end(); ++it) {
                QStringList nodes;
                if (it.value().isArray()) {
                    const QJsonArray arr = it.value().toArray();
                    for (const QJsonValue& value : arr) {
                        nodes << value.toString();
                    }
                } else {
                    nodes = splitNodeList(it.value().toString());
                }
                addZoneNodes(m_ruviewNodeZoneMap, m_ruviewAlertZones, it.key(), nodes);
            }
        }
    }

    const QRegularExpressionMatch zone1Match = zone1Re.match(text);
    if (zone1Match.hasMatch()) {
        addZoneNodes(m_ruviewNodeZoneMap, m_ruviewAlertZones, "zone1", splitNodeList(zone1Match.captured(1)));
    }

    const QRegularExpressionMatch zone2Match = zone2Re.match(text);
    if (zone2Match.hasMatch()) {
        addZoneNodes(m_ruviewNodeZoneMap, m_ruviewAlertZones, "zone2", splitNodeList(zone2Match.captured(1)));
    }

}

void MainWindow::maybeRaiseRuViewZoneAlert(const MqttEvent& ev)
{
    if (!ev.state || !isRuViewEvent(ev)) return;
    const QString level = ev.level.trimmed().toLower();
    if (level != "node" && level != "fused") return;

    const QString sensorId = normalizeZoneKey(ev.sensorId);
    QString zone = normalizeZoneKey(ev.zone);
    const QString mappedZone = sensorId.isEmpty() ? QString() : m_ruviewNodeZoneMap.value(sensorId);

    if (zone.isEmpty()) zone = mappedZone;
    if (zone.isEmpty()) return;
    if (!mappedZone.isEmpty() && zone != mappedZone) {
        return;
    }
    if (zone != "zone1" && zone != "zone2") return;
    if (zone == "zone1") {
        if (!(level == "node" && sensorId == "rx1") &&
            !(level == "fused" && sensorId == "fusion-engine")) {
            return;
        }
    } else if (zone == "zone2") {
        if (!(level == "fused" && sensorId == "fusion-engine")) {
            return;
        }
    }
    if (!m_ruviewAlertZones.isEmpty() && !m_ruviewAlertZones.contains(zone)) return;

    const QDateTime now = QDateTime::currentDateTimeUtc();
    const QDateTime recentCctv = m_recentCctvZoneSeenUtc.value(zone);
    if (recentCctv.isValid() && recentCctv.msecsTo(now) <= 3000) return;

    const QDateTime recentAlert = m_recentRuviewAlertUtc.value(zone);
    if (recentAlert.isValid() && recentAlert.msecsTo(now) <= 5000) return;
    m_recentRuviewAlertUtc.insert(zone, now);

    const QString sensorLabel = ev.sensorId.isEmpty() ? "unknown" : ev.sensorId;
    const QString levelLabel = ev.level.isEmpty() ? ev.topic : ev.level;
    const QString text = QString("RuView detected %1 (mapped %2, %3) without recent CCTV IvaArea confirmation.")
                             .arg(sensorLabel, displayRuviewZoneLabel(zone), levelLabel);

    appendLog(QString("[RUVIEW ALERT] %1").arg(text));

    auto* box = new QMessageBox(QMessageBox::Warning,
                                "RuView Alert",
                                text,
                                QMessageBox::Ok,
                                this);
    box->setAttribute(Qt::WA_DeleteOnClose, true);
    box->show();
    box->raise();
    box->activateWindow();
}

void MainWindow::applyRuViewZoneVisual(bool present, double confidence)
{
    const double c = std::max(0.0, std::min(1.0, confidence));
    if (ui && ui->labelCentralMap) ui->labelCentralMap->setZonePresenceState(present, c);
    if (auto* capMap = findChild<MiniMapWidget*>("labelCentralCapture")) {
        capMap->setZonePresenceState(present, c);
    }
}

bool MainWindow::publishRuViewControl(bool enabled)
{
    if (!m_pub) return false;

    QJsonObject payload;
    payload["type"] = "command";
    payload["target"] = "vision_pose_bridge";
    payload["command"] = "set_enabled";
    payload["value"] = enabled;
    payload["source"] = "qt_main_window";
    payload["ts"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);

    return m_pub->publishJson(
        QString::fromUtf8(kVisionPoseBridgeCommandTopic),
        QJsonDocument(payload).toJson(QJsonDocument::Compact),
        1,
        false
    );
}

bool MainWindow::publishTankControl(const QString& group, const QString& command, bool active)
{
    if (!m_pub) {
        appendLog(QString("Tank control skipped: publisher missing (%1/%2)").arg(group, command));
        if (m_tankControlStatusLabel) m_tankControlStatusLabel->setText("MQTT offline");
        return false;
    }

    QJsonObject payload;
    payload["type"] = "tank_control";
    payload["target"] = "tank";
    payload["group"] = group;
    payload["command"] = command;
    payload["active"] = active;
    payload["source"] = "qt_main_window";
    payload["ts_ms"] = QDateTime::currentMSecsSinceEpoch();

    const bool ok = m_pub->publishJson(
        QString::fromUtf8(kTankControlTopic),
        QJsonDocument(payload).toJson(QJsonDocument::Compact),
        1,
        false);

    if (ok) {
        appendLog(QString("Tank control published: %1/%2 active=%3")
                      .arg(group, command, active ? "true" : "false"));
        if (m_tankControlStatusLabel) {
            m_tankControlStatusLabel->setText(
                active ? QString("%1: %2").arg(group.toUpper(), command)
                       : QString("%1: idle").arg(group.toUpper()));
        }
    } else {
        appendLog(QString("Tank control publish failed: %1/%2").arg(group, command));
        if (m_tankControlStatusLabel) m_tankControlStatusLabel->setText("Publish failed");
    }

    return ok;
}

void MainWindow::bindTankControlButton(QPushButton* button, const QString& group, const QString& command)
{
    if (!button) return;

    connect(button, &QPushButton::pressed, this, [this, group, command]() {
        publishTankControl(group, command, true);
    });
    connect(button, &QPushButton::released, this, [this, group, command]() {
        publishTankControl(group, command, false);
    });
}

void MainWindow::syncRuViewToggle(bool enabled)
{
    if (auto* ruviewBtn = findChild<QPushButton*>("ruviewbtn")) {
        const QSignalBlocker blocker(ruviewBtn);
        ruviewBtn->setChecked(enabled);
    }
}

void MainWindow::updateStatusBarText()
{
    QString ruviewText = "RuView: UI OFF";
    QString cardStatus = "RuView Status: UI OFF";
    QString cardState = "State: -";
    QString cardConf = "Confidence: -";
    QString cardNodes = "Detected/Active Nodes: -";

    if (m_ruviewUiEnabled && m_ruviewOnline) {
        const QString stateText = m_ruviewPresent ? "detected" : "cleared";
        ruviewText = QString("RuView: ONLINE | %1 | conf=%2 | nodes=%3/%4")
                         .arg(stateText)
                         .arg(m_ruviewConfidence, 0, 'f', 2)
                         .arg(m_ruviewDetectedNodes)
                         .arg(m_ruviewActiveNodes);
        cardStatus = "RuView Status: ONLINE";
        cardState = QString("State: %1").arg(stateText);
        cardConf = QString("Confidence: %1").arg(m_ruviewConfidence, 0, 'f', 2);
        cardNodes = QString("Detected/Active Nodes: %1/%2").arg(m_ruviewDetectedNodes).arg(m_ruviewActiveNodes);
    } else if (m_ruviewUiEnabled) {
        ruviewText = "RuView: OFFLINE";
        cardStatus = "RuView Status: OFFLINE";
    }

    if (m_ruviewStatusLabel) m_ruviewStatusLabel->setText(cardStatus);
    if (m_ruviewStateLabel) m_ruviewStateLabel->setText(cardState);
    if (m_ruviewConfLabel) m_ruviewConfLabel->setText(cardConf);
    if (m_ruviewNodesLabel) m_ruviewNodesLabel->setText(cardNodes);
    updateHeaderSummary();

    statusBar()->showMessage(m_statusBaseText + " | " + ruviewText);
}

// ------------- MQTT service ------------------
void MainWindow::onMqttLogLine(const QString& s)
{
    if (s.trimmed().isEmpty()) return;
    appendLog(s);
}

void MainWindow::onMqttEvent(const MqttEvent& ev)
{
    if (ev.hasRobotStatus) {
        m_lastRobotStatusEvent = ev;
        if (m_hasTarget) {
            m_lastRobotStatusEvent.rcTargetX = m_targetWorld.x();
            m_lastRobotStatusEvent.rcTargetY = m_targetWorld.y();
            m_lastRobotStatusEvent.rcTargetZ = 0.0;
        }
        if (m_robotConnLabel) {
            m_robotConnLabel->setText(ev.rcConnected ? "ONLINE" : "OFFLINE");
            m_robotConnLabel->setProperty("robotConn", ev.rcConnected ? "online" : "offline");
            m_robotConnLabel->style()->unpolish(m_robotConnLabel);
            m_robotConnLabel->style()->polish(m_robotConnLabel);
        }
        if (m_robotModeLabel) m_robotModeLabel->setText(ev.rcMode.isEmpty() ? "-" : ev.rcMode.toUpper());
        if (m_robotMissionLabel) m_robotMissionLabel->setText(ev.rcMission.isEmpty() ? "-" : ev.rcMission);
        if (m_robotPoseLabel) {
            m_robotPoseLabel->setText(
                QString("x %1 / y %2 / h %3")
                    .arg(ev.rcX, 0, 'f', 1)
                    .arg(ev.rcY, 0, 'f', 1)
                    .arg(ev.rcHeading, 0, 'f', 0));
        }
        if (m_robotSpeedLabel) {
            m_robotSpeedLabel->setText(QString("%1 m/s").arg(ev.rcSpeed, 0, 'f', 2));
        }
        if (m_robotCpuValueLabel) {
            if (ev.rcCpuUsage >= 0.0) {
                const double cpu = std::clamp(ev.rcCpuUsage, 0.0, 100.0);
                m_robotCpuValueLabel->setText(QString("%1%").arg(cpu, 0, 'f', 0));
                QString level;
                if (cpu >= 85.0) level = "danger";
                else if (cpu >= 65.0) level = "warn";
                m_robotCpuValueLabel->setProperty("level", level);
            } else {
                m_robotCpuValueLabel->setText("- %");
                m_robotCpuValueLabel->setProperty("level", QString());
            }
            m_robotCpuValueLabel->style()->unpolish(m_robotCpuValueLabel);
            m_robotCpuValueLabel->style()->polish(m_robotCpuValueLabel);
        }
        if (m_robotLastSeenLabel) {
            m_robotLastSeenLabel->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));
        }
        if (m_robotStatusWindow) {
            m_robotStatusWindow->setRobotStatus(m_lastRobotStatusEvent);
        }
        return;
    }

    if (ev.topic == "MotionAlarm" || ev.topic == "MotionDetection")
        return;

    const bool ruview = isRuViewEvent(ev);
    const QString ruviewLevel = ev.level.trimmed().toLower();
    if (ruview && (ruviewLevel == "node" || ruviewLevel == "fused")) {
        Q_UNUSED(ruviewLevel);
    }
    const QString normalizedZone = normalizeZoneKey(ev.zone);
    if (!ruview &&
        ev.state &&
        ev.topic.compare("IvaArea", Qt::CaseInsensitive) == 0 &&
        !normalizedZone.isEmpty()) {
        m_recentCctvZoneSeenUtc.insert(normalizedZone, QDateTime::currentDateTimeUtc());
    }

    const bool isHumanDetectionEvent =
        ev.src.compare("cctv", Qt::CaseInsensitive) == 0 &&
        ev.topic.compare("ObjectDetection", Qt::CaseInsensitive) == 0 &&
        ev.objectType.compare("Human", Qt::CaseInsensitive) == 0;
    if (isHumanDetectionEvent && !m_humanBoxEnabled) {
        return;
    }

    if (m_humanBoxOverlay) {
        const bool isHumanBoxEvent = isHumanDetectionEvent;

        if (isHumanBoxEvent) {
            const QDateTime now = QDateTime::currentDateTimeUtc();
            const QString key = !ev.objectId.isEmpty()
                ? ev.objectId
                : QString("%1_%2_%3_%4_%5")
                      .arg(ev.utc)
                      .arg(ev.bbox.left())
                      .arg(ev.bbox.top())
                      .arg(ev.bbox.right())
                      .arg(ev.bbox.bottom());

            if (ev.state && ev.bbox.isValid()) {
                const QSize minBounds(ev.bbox.right() + 1, ev.bbox.bottom() + 1);
                QSize videoFrameSize;
                if (ui && ui->labelCentralView) {
                    videoFrameSize = ui->labelCentralView->currentFrameSize();
                }
                const QSize explicitSourceSize(ev.frameW, ev.frameH);
                const QSize nextSourceSize =
                    chooseDetectionSourceSize(minBounds, explicitSourceSize, videoFrameSize, m_humanBoxSourceSize);
                if (nextSourceSize != m_humanBoxSourceSize) {
                    m_humanBoxSourceSize = nextSourceSize;
                    m_humanBoxOverlay->setBoxSourceSize(m_humanBoxSourceSize);
                }
                m_humanBoxOverlay->setBoxSourceOffset(QPointF(
                    m_humanBoxSourceSize.width() * kHumanBoxOffsetXRatio,
                    m_humanBoxSourceSize.height() * kHumanBoxOffsetYRatio));
                m_humanBoxOverlay->setBoxPerspectiveOffset(QPointF(
                    m_humanBoxSourceSize.width() * kHumanBoxPerspectiveOffsetXRatio,
                    0.0));
                if (shouldAcceptHumanBox(ev, m_humanBoxSourceSize)) {
                    const QRect nextBox = smoothHumanBox(m_humanBoxSmoothed.value(key), ev.bbox);
                    m_humanBoxSmoothed.insert(key, QRectF(nextBox));
                    m_humanBoxes.insert(key, nextBox);
                    m_humanBoxSeenUtc.insert(key, now);
                } else {
                    m_humanBoxes.remove(key);
                    m_humanBoxSmoothed.remove(key);
                    m_humanBoxSeenUtc.remove(key);
                }
            } else {
                m_humanBoxes.remove(key);
                m_humanBoxSmoothed.remove(key);
                m_humanBoxSeenUtc.remove(key);
            }

            QStringList staleKeys;
            for (auto it = m_humanBoxSeenUtc.cbegin(); it != m_humanBoxSeenUtc.cend(); ++it) {
                if (it.value().msecsTo(now) > kHumanBoxKeepAliveMs) staleKeys.push_back(it.key());
            }
            for (const QString& staleKey : staleKeys) {
                m_humanBoxSeenUtc.remove(staleKey);
                m_humanBoxes.remove(staleKey);
                m_humanBoxSmoothed.remove(staleKey);
            }

            QVector<QPair<QDateTime, QString>> recentKeys;
            recentKeys.reserve(m_humanBoxSeenUtc.size());
            for (auto it = m_humanBoxSeenUtc.cbegin(); it != m_humanBoxSeenUtc.cend(); ++it) {
                recentKeys.push_back(qMakePair(it.value(), it.key()));
            }
            std::sort(recentKeys.begin(), recentKeys.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });

            QVector<QRect> boxes;
            boxes.reserve(std::min(static_cast<int>(recentKeys.size()), kHumanBoxMaxVisible));
            for (const auto& entry : recentKeys) {
                const QRect box = m_humanBoxes.value(entry.second);
                if (box.isValid()) boxes.push_back(box);
                if (boxes.size() >= kHumanBoxMaxVisible) break;
            }

            const bool shouldRenderNow =
                !m_lastHumanBoxRenderUtc.isValid() ||
                m_lastHumanBoxRenderUtc.msecsTo(now) >= kHumanBoxRenderThrottleMs ||
                !m_humanBoxEnabled ||
                boxes.isEmpty();
            if (shouldRenderNow) {
                if (ui && ui->labelCentralView) {
                    QSize sourceSize = ui->labelCentralView->currentFrameSize();
                    if (!sourceSize.isValid() ||
                        sourceSize.width() < m_humanBoxSourceSize.width() ||
                        sourceSize.height() < m_humanBoxSourceSize.height()) {
                        sourceSize = m_humanBoxSourceSize;
                    }
                    m_humanBoxOverlay->setVideoGeometry(
                        ui->labelCentralView->videoDisplayRect(),
                        sourceSize
                    );
                }
                m_humanBoxOverlay->setHumanBoxes(boxes, m_humanBoxEnabled && !boxes.isEmpty());
                m_lastHumanBoxRenderUtc = now;
            }
        }
    }

    if (ev.messageType.compare(kVisionPoseBridgeStatusType, Qt::CaseInsensitive) == 0 && ev.hasEnabled) {
        m_ruviewUiEnabled = ev.enabled;
        syncRuViewToggle(ev.enabled);

        if (!ev.enabled) {
            clearRuViewUiState();
        } else {
            m_ruviewOnline = true;
            m_ruviewLastSeenUtc = QDateTime::currentDateTimeUtc();
        }

        updateStatusBarText();
        refreshEventFilter();
        saveUiSettings();
        rebuildPrimaryNavList();
        return;
    }

    if (ruview && !m_ruviewUiEnabled) return;
    const bool isIvaAreaEvent =
        ev.src.compare("cctv", Qt::CaseInsensitive) == 0 &&
        ev.topic.compare("IvaArea", Qt::CaseInsensitive) == 0;
    if (!ev.state && !ruview && !(isIvaAreaEvent && !ev.clipUrl.isEmpty())) return;

    if (ruview) {
        m_ruviewOnline = true;
        m_ruviewPresent = ev.state;
        m_ruviewConfidence = std::max(0.0, std::min(1.0, ev.confidence));
        m_ruviewActiveNodes = ev.activeNodes;
        m_ruviewDetectedNodes = ev.detectedNodes;
        m_ruviewLastSeenUtc = QDateTime::currentDateTimeUtc();
        applyRuViewZoneVisual(ev.state, ev.confidence);
        updateStatusBarText();
        if (!ev.state) return;
        maybeRaiseRuViewZoneAlert(ev);
    }

    if (isHumanDetectionEvent) {
        const QDateTime now = QDateTime::currentDateTimeUtc();
        if (m_lastHumanEventListUtc.isValid() &&
            m_lastHumanEventListUtc.msecsTo(now) < kHumanEventListThrottleMs) {
            return;
        }
        m_lastHumanEventListUtc = now;
    }

    if (m_eventListPaused) return;

    QString utcShort = ev.utc;
    int dot = utcShort.indexOf('.');
    if (dot > 0) utcShort = utcShort.left(dot);
    utcShort.replace("Z", "");
    utcShort.replace("T", " ");

    QString src = ev.src.isEmpty() ? "unknown" : ev.src;
    if (ruview) src = "ruview";
    QString srcLabel = src;
    if (ruview) {
        if (ev.level.compare("fused", Qt::CaseInsensitive) == 0) srcLabel = "ruview:fused";
        else if (ev.level.compare("node", Qt::CaseInsensitive) == 0) srcLabel = "ruview:node";
        else srcLabel = "ruview";
    }

    EventItem item;
    item.source = "mqtt";
    item.rawClip.clear();
    item.privacyClip.clear();
    item.wardName.clear();
    item.id = QString("mqtt-%1").arg(++m_mqttSeq);

    const int MAX_EVENTS = 2000;
    if (m_events.size() >= MAX_EVENTS) {
        m_events.clear();
        ui->listEvent->clear();
        appendLog("Event list cleared (MAX_EVENTS reached)");
    }

    m_events.push_back(item);
    const int idx = m_events.size() - 1;

    const QString locationLabel = (ruview && !ev.sensorId.isEmpty())
        ? ev.sensorId
        : (!ev.zone.isEmpty() && ruview ? displayRuviewZoneLabel(ev.zone)
                                        : (!ev.zone.isEmpty() ? ev.zone : ev.cam));
    const QString firstLine = locationLabel.isEmpty()
        ? ev.topic
        : QString("%1  %2").arg(ev.topic, locationLabel);
    QStringList second;
    second << srcLabel;
    if (!ev.sensorId.isEmpty() && !ruview) second << ev.sensorId;
    if (!ev.rule.isEmpty())   second << ev.rule;
    if (!ev.action.isEmpty()) second << ev.action;
    if (ruview) {
        if (!ev.sensorId.isEmpty()) second << QString("sensor=%1").arg(ev.sensorId);
        second << QString("conf=%1").arg(ev.confidence, 0, 'f', 2);
        if (ev.activeNodes > 0 || ev.detectedNodes > 0) {
            second << QString("nodes=%1/%2").arg(ev.detectedNodes).arg(ev.activeNodes);
        }
    }
    const QString secondLine = second.join(" | ");
    const QString dedupKey = buildEventDedupKey(ev, srcLabel, locationLabel);
    QColor itemColor("#cfd8ea");
    if (ruview) {
        const bool alertZone = isRuviewAlertZoneKey(ev.zone);
        itemColor = QColor(150, 150, 150);
        if (ev.state && alertZone) {
            itemColor = QColor(255, 90, 90);
            if (ev.level.compare("node", Qt::CaseInsensitive) == 0) {
                itemColor = QColor(230, 150, 60);
            }
        }
    } else {
        auto it = m_eventColor.find(ev.topic);
        if (it != m_eventColor.end()) {
            itemColor = it.value();
        }
    }
    if (tryMergeDuplicateEvent(dedupKey, firstLine, secondLine, ev, src, utcShort, itemColor)) {
        updateEventCountBadge();
        if (m_eventAutoSortEnabled) ui->listEvent->scrollToTop();
        else ui->listEvent->scrollToBottom();
    } else {
        auto* lw = new QListWidgetItem(formatEventItemText(firstLine, secondLine, 1));
        lw->setSizeHint(QSize(0, 58));
        lw->setData(Qt::UserRole, idx);
        lw->setData(Qt::UserRole + 1, ev.clipUrl);
        lw->setData(Qt::UserRole + 2, ev.clipSec > 0 ? ev.clipSec : 5);
        lw->setData(Qt::UserRole + 3, ev.topic);
        lw->setData(Qt::UserRole + 4, ev.cam);
        lw->setData(Qt::UserRole + 5, utcShort);
        lw->setData(Qt::UserRole + 6, src);
        lw->setData(Qt::UserRole + 7, ev.level);
        lw->setData(Qt::UserRole + 10, dedupKey);
        lw->setData(Qt::UserRole + 11, QDateTime::currentMSecsSinceEpoch());
        lw->setData(Qt::UserRole + 12, firstLine);
        lw->setData(Qt::UserRole + 13, secondLine);
        lw->setData(Qt::UserRole + 14, 1);
        lw->setForeground(QBrush(itemColor));
        lw->setText(formatEventItemText(firstLine, secondLine, 1));
        if (m_eventAutoSortEnabled) ui->listEvent->insertItem(0, lw);
        else ui->listEvent->addItem(lw);
        refreshEventItemWidget(lw);
        lw->setHidden(!shouldShowEventItem(lw));
        updateEventCountBadge();
        if (m_eventAutoSortEnabled) ui->listEvent->scrollToTop();
        else ui->listEvent->scrollToBottom();
    }
    if (m_autoClipPopupEnabled && isIvaAreaEvent && !ev.clipUrl.isEmpty()) {
        if (m_clipPopup && m_clipPopup->isVisible()) {
            return;
        }
        if (!m_clipPopup) {
            m_clipPopup = new ClipPopup(this);
            m_clipPopup->setAttribute(Qt::WA_DeleteOnClose, false);
        }

        m_clipPopup->show();
        m_clipPopup->raise();
        m_clipPopup->activateWindow();

        const int clipSec = (ev.clipSec > 0) ? ev.clipSec : 5;

        m_clipPopup->playClip(
            ev.cam,
            ev.topic,
            utcShort,
            QUrl(ev.clipUrl),
            clipSec
            );
    }
}

void MainWindow::onHomographyReceived(const QTransform& hImgToWorld)
{
    bool ok = false;
    const QTransform hWorldToImg = hImgToWorld.inverted(&ok);
    if (!ok) {
        appendLog("Homography invert failed");
        return;
    }

    if (m_homographyLocked) {
        return;
    }

    m_hWorldToImg = hWorldToImg;
    m_hasHomography = true;
    m_homographyLocked = true; // apply once, ignore repeated publish
    m_homographyFromMqttImage = true;
    if (m_overlay) {
        m_homographyBaseSize = m_overlay->size();
    }
    if (m_captureOverlay) m_captureOverlay->setHomography(m_hWorldToImg, true);
    if (m_calibImgPts.size() != m_calibWorldPts.size()) {
        m_calibImgPts.clear();
        m_calibImgPts.reserve(m_calibWorldPts.size());
        for (const auto& w : m_calibWorldPts)
            m_calibImgPts.push_back(m_hWorldToImg.map(w));
    }
    applyOverlayTransform();
}

void MainWindow::onMapReceived(const MapData& map)
{
    QVector<MapNode> nodes;
    nodes.reserve(map.nodes.size());
    for (const auto& n : map.nodes) nodes.push_back({n.id, n.pos});

    QVector<MapEdge> edges;
    edges.reserve(map.edges.size());
    for (const auto& e : map.edges) edges.push_back({e.from, e.to});

    QVector<QPointF> polyline = closedPolylineFromMap(map.polyline);

    if (!nodes.isEmpty()) {
        double minX = nodes[0].pos.x();
        double maxX = nodes[0].pos.x();
        double minY = nodes[0].pos.y();
        double maxY = nodes[0].pos.y();
        for (const auto& n : nodes) {
            minX = std::min(minX, n.pos.x());
            maxX = std::max(maxX, n.pos.x());
            minY = std::min(minY, n.pos.y());
            maxY = std::max(maxY, n.pos.y());
        }
        const double worldW = std::max(1.0, (maxX - minX) + 4.0);
        const double worldH = std::max(1.0, (maxY - minY) + 4.0);
        if (ui && ui->labelCentralMap) {
            ui->labelCentralMap->setWorldSize(worldW, worldH);
            ui->labelCentralMap->setNodes(nodes);
            ui->labelCentralMap->setEdges(edges);
            ui->labelCentralMap->setPolyline(polyline);
        }
        if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
            rcMap->setWorldSize(worldW, worldH);
            rcMap->setNodes(nodes);
            rcMap->setEdges(edges);
            rcMap->setPolyline(polyline);
        }
        if (auto* capMap = findChild<MiniMapWidget*>("labelCentralCapture")) {
            capMap->setWorldSize(worldW, worldH);
            capMap->setNodes(nodes);
            capMap->setEdges(edges);
            capMap->setPolyline(polyline);
        }
    }

    if (m_overlay) {
        QVector<OverlayNode> overlayNodes;
        overlayNodes.reserve(map.nodes.size());
        for (const auto& n : map.nodes) overlayNodes.push_back({n.id, n.pos});
        m_overlay->setNodes(overlayNodes);
        m_overlay->setNodesVisible(m_nodeOverlayEnabled);
        m_overlay->setPolyline(polyline);
    }

    if (map.nodes.size() >= 4) {
        auto pointFor = [&map](const QString& id, QPointF& out) -> bool {
            for (const auto& n : map.nodes) {
                if (n.id == id) {
                    out = n.pos;
                    return true;
                }
            }
            return false;
        };

        QPointF p10, p11, p12, p13;
        if (pointFor("10", p10) && pointFor("11", p11) &&
            pointFor("12", p12) && pointFor("13", p13)) {
            m_calibWorldPts = {p10, p11, p12, p13};
            if (m_captureOverlay) m_captureOverlay->setWorldPolygon(buildCalibPolyline(m_calibWorldPts));
            if (m_calibImgPts.size() == 4) {
                rebuildHomographyFromCalib();
                saveOverlayTweak();
                publishHomographyImageToWorld();
            }
        }
    }
}

void MainWindow::initLocalMapFallback()
{
    if (!ui->labelCentralMap) return;

    // User-provided world coordinates from ArUco IDs 10, 11, 12, 13.
    const QVector<MapNode> nodes = {
        {"10", QPointF(1.0, 330.0)},
        {"11", QPointF(95.0, 330.0)},
        {"12", QPointF(1.0, 1.0)},
        {"13", QPointF(95.0, 1.0)}
    };
    const QVector<MapEdge> edges = {
        {"10", "11"},
        {"11", "13"},
        {"13", "12"},
        {"12", "10"}
    };
    const QVector<QPointF> polyline = {
        QPointF(1.0, 1.0),
        QPointF(95.0, 1.0),
        QPointF(95.0, 330.0),
        QPointF(1.0, 330.0),
        QPointF(1.0, 1.0)
    };
    m_calibWorldPts = {
        QPointF(1.0, 330.0),
        QPointF(95.0, 330.0),
        QPointF(1.0, 1.0),
        QPointF(95.0, 1.0)
    };

    const double worldW = 100.0;
    const double worldH = 334.0;
    ui->labelCentralMap->setWorldSize(worldW, worldH);
    ui->labelCentralMap->setNodes(nodes);
    ui->labelCentralMap->setEdges(edges);
    ui->labelCentralMap->setPolyline(polyline);
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->setWorldSize(worldW, worldH);
        rcMap->setNodes(nodes);
        rcMap->setEdges(edges);
        rcMap->setPolyline(polyline);
    }
    if (auto* capMap = findChild<MiniMapWidget*>("labelCentralCapture")) {
        capMap->setWorldSize(worldW, worldH);
        capMap->setNodes(nodes);
        capMap->setEdges(edges);
        capMap->setPolyline(polyline);
    }

    if (m_overlay) {
        QVector<OverlayNode> overlayNodes;
        overlayNodes.reserve(nodes.size());
        for (const auto& n : nodes) overlayNodes.push_back({ n.id, n.pos });
        m_overlay->setNodes(overlayNodes);
        m_overlay->setNodesVisible(m_nodeOverlayEnabled);
        m_overlay->setPolyline(polyline);
        m_overlay->setMapVisible(m_mapOverlayEnabled);
    }

    appendLog("Local map fallback loaded (10/11/12/13)");
}

void MainWindow::applyOverlayTransform()
{
    if (!m_overlay || !m_hasHomography) return;
    m_overlay->setHomography(currentOverlayHomography());
}

void MainWindow::saveOverlayTweak() const
{
    QSettings s("overlay_calib.ini", QSettings::IniFormat);
    s.setValue("overlay/dx", m_tweakDx);
    s.setValue("overlay/dy", m_tweakDy);
    s.setValue("overlay/scale", m_tweakScale);
    s.setValue("overlay/rot_deg", m_tweakRotDeg);
    const int baseW = (m_homographyBaseSize.width() > 0)
        ? m_homographyBaseSize.width()
        : (m_overlay ? m_overlay->width() : 0);
    const int baseH = (m_homographyBaseSize.height() > 0)
        ? m_homographyBaseSize.height()
        : (m_overlay ? m_overlay->height() : 0);
    s.setValue("overlay/base_w", baseW);
    s.setValue("overlay/base_h", baseH);
    s.setValue("overlay/calib_count", m_calibImgPts.size());
    for (int i = 0; i < m_calibImgPts.size(); ++i) {
        s.setValue(QString("overlay/calib_%1_x").arg(i), m_calibImgPts[i].x());
        s.setValue(QString("overlay/calib_%1_y").arg(i), m_calibImgPts[i].y());
    }
}

void MainWindow::loadOverlayTweak()
{
    QSettings s("overlay_calib.ini", QSettings::IniFormat);
    m_tweakDx = s.value("overlay/dx", 0.0).toDouble();
    m_tweakDy = s.value("overlay/dy", 0.0).toDouble();
    m_tweakScale = s.value("overlay/scale", 1.0).toDouble();
    m_tweakRotDeg = s.value("overlay/rot_deg", 0.0).toDouble();
    m_homographyBaseSize = QSize(
        s.value("overlay/base_w", 0).toInt(),
        s.value("overlay/base_h", 0).toInt());
    const int n = s.value("overlay/calib_count", 0).toInt();
    if (n == 4) {
        m_calibImgPts.clear();
        for (int i = 0; i < n; ++i) {
            const double x = s.value(QString("overlay/calib_%1_x").arg(i), 0.0).toDouble();
            const double y = s.value(QString("overlay/calib_%1_y").arg(i), 0.0).toDouble();
            m_calibImgPts.push_back(QPointF(x, y));
        }
        if (m_calibWorldPts.size() == 4) {
            rebuildHomographyFromCalib();
            m_homographyLocked = true;
            appendLog("Saved capture calibration restored (homography locked)");
        }
    }
}

int MainWindow::findNearestCalibPoint(const QPointF& p, double thPx) const
{
    if (m_calibImgPts.size() != 4) return -1;
    int best = -1;
    double bestD = thPx;
    for (int i = 0; i < m_calibImgPts.size(); ++i) {
        const double d = QLineF(p, m_calibImgPts[i]).length();
        if (d <= bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

void MainWindow::rebuildHomographyFromCalib()
{
    if (m_calibWorldPts.size() != 4 || m_calibImgPts.size() != 4) return;
    QTransform h;
    if (!HomographyCalib::computeWorldToImage(m_calibWorldPts, m_calibImgPts, h, false, 3.0)) {
        appendLog("Calibration solve failed");
        return;
    }
    m_hWorldToImg = h;
    m_hasHomography = true;
    m_homographyFromMqttImage = false;
    if (m_overlay) {
        m_homographyBaseSize = m_overlay->size();
    }
    if (m_captureOverlay) m_captureOverlay->setHomography(m_hWorldToImg, true);
    applyOverlayTransform();
}

QTransform MainWindow::currentOverlayHomography() const
{
    if (!m_overlay) return m_hWorldToImg;

    if (m_homographyFromMqttImage) {
        auto* video = ui ? ui->labelCentralView : nullptr;
        if (video) {
            const QSize frameSize = video->currentFrameSize();
            const QRect drawRect = video->videoDisplayRect();
            if (frameSize.width() > 0 && frameSize.height() > 0 &&
                drawRect.width() > 0 && drawRect.height() > 0) {
                QTransform imageToWidget;
                imageToWidget.translate(drawRect.left(), drawRect.top());
                imageToWidget.scale(
                    static_cast<double>(drawRect.width()) / static_cast<double>(frameSize.width()),
                    static_cast<double>(drawRect.height()) / static_cast<double>(frameSize.height()));
                return imageToWidget * m_hWorldToImg;
            }
        }
    }

    QTransform tweak;
    tweak.translate(m_tweakDx, m_tweakDy);

    const double cx = m_overlay->width() * 0.5;
    const double cy = m_overlay->height() * 0.5;
    tweak.translate(cx, cy);
    tweak.rotate(m_tweakRotDeg);
    tweak.scale(m_tweakScale, m_tweakScale);
    tweak.translate(-cx, -cy);

    const int baseW = (m_homographyBaseSize.width() > 0) ? m_homographyBaseSize.width() : m_overlay->width();
    const int baseH = (m_homographyBaseSize.height() > 0) ? m_homographyBaseSize.height() : m_overlay->height();
    const double sx = baseW > 0 ? static_cast<double>(m_overlay->width()) / static_cast<double>(baseW) : 1.0;
    const double sy = baseH > 0 ? static_cast<double>(m_overlay->height()) / static_cast<double>(baseH) : 1.0;

    QTransform resizeScale;
    resizeScale.scale(sx, sy);
    return resizeScale * tweak * m_hWorldToImg;
}

void MainWindow::publishHomographyImageToWorld()
{
    if (!m_pub) {
        appendLog("Homography publish skipped: publisher is null");
        return;
    }
    if (!m_hasHomography) {
        appendLog("Homography publish skipped: homography unavailable");
        return;
    }

    bool ok = false;
    const QTransform hImgToWorld = m_hWorldToImg.inverted(&ok);
    if (!ok) {
        appendLog("Homography publish failed: inversion failed");
        return;
    }

    QJsonObject o;
    o["type"] = "homography";
    o["key"] = "H_img2world";
    o["rows"] = 3;
    o["cols"] = 3;
    o["src"] = "qt-main-window";
    o["ts_ms"] = QDateTime::currentMSecsSinceEpoch();

    QJsonArray data;
    data.append(hImgToWorld.m11());
    data.append(hImgToWorld.m21());
    data.append(hImgToWorld.m31());
    data.append(hImgToWorld.m12());
    data.append(hImgToWorld.m22());
    data.append(hImgToWorld.m32());
    data.append(hImgToWorld.m13());
    data.append(hImgToWorld.m23());
    data.append(hImgToWorld.m33());
    o["data"] = data;

    const QByteArray payload = QJsonDocument(o).toJson(QJsonDocument::Compact);
    const bool pubOk = m_pub->publishJson("wiserisk/calib/homography", payload, 1, false);
    appendLog(pubOk
                  ? "Homography published: wiserisk/calib/homography"
                  : "Homography publish failed: wiserisk/calib/homography");
}
void MainWindow::onOverlayCalibPressed(const QPointF& p)
{
    m_dragCalibIdx = findNearestCalibPoint(p);
}

void MainWindow::onOverlayCalibMoved(const QPointF& p)
{
    if (m_dragCalibIdx < 0 || m_dragCalibIdx >= m_calibImgPts.size()) return;
    m_calibImgPts[m_dragCalibIdx] = p;
    rebuildHomographyFromCalib();
}

void MainWindow::onOverlayCalibReleased(const QPointF& p)
{
    if (m_dragCalibIdx >= 0 && m_dragCalibIdx < m_calibImgPts.size()) {
        m_calibImgPts[m_dragCalibIdx] = p;
        rebuildHomographyFromCalib();
        saveOverlayTweak();
        appendLog(QString("Calib point %1 updated and saved").arg(m_dragCalibIdx));
    }
    m_dragCalibIdx = -1;
}

void MainWindow::onMiniMapNodeMoved(const QString& id, const QPointF& worldPos)
{
    if (id == "10" && m_calibWorldPts.size() >= 1) m_calibWorldPts[0] = worldPos;
    else if (id == "11" && m_calibWorldPts.size() >= 2) m_calibWorldPts[1] = worldPos;
    else if (id == "12" && m_calibWorldPts.size() >= 3) m_calibWorldPts[2] = worldPos;
    else if (id == "13" && m_calibWorldPts.size() >= 4) m_calibWorldPts[3] = worldPos;

    if (m_overlay && m_calibWorldPts.size() >= 4) {
        QVector<OverlayNode> ons = {
            {"10", m_calibWorldPts[0]},
            {"11", m_calibWorldPts[1]},
            {"12", m_calibWorldPts[2]},
            {"13", m_calibWorldPts[3]}
        };
        QVector<QPointF> poly = buildCalibPolyline(m_calibWorldPts);
        m_overlay->setNodes(ons);
        m_overlay->setNodesVisible(m_nodeOverlayEnabled);
        m_overlay->setPolyline(poly);
        if (m_captureOverlay) m_captureOverlay->setWorldPolygon(poly);
    }

    if (m_calibImgPts.size() == 4) {
        rebuildHomographyFromCalib();
        saveOverlayTweak();
    }

    if (m_hasHomography) {
        const QPointF img = m_hWorldToImg.map(worldPos);
        appendLog(QString("Node %1 world=(%2,%3) -> image=(%4,%5)")
                      .arg(id)
                      .arg(worldPos.x(), 0, 'f', 3)
                      .arg(worldPos.y(), 0, 'f', 3)
                      .arg(img.x(), 0, 'f', 1)
                      .arg(img.y(), 0, 'f', 1));
    } else {
        appendLog(QString("Node %1 world=(%2,%3)")
                      .arg(id)
                      .arg(worldPos.x(), 0, 'f', 3)
                      .arg(worldPos.y(), 0, 'f', 3));
    }
}

void MainWindow::onMiniMapWorldClicked(const QPointF& worldPos)
{
    if (!m_patrolPointsEnabled) {
        appendLog("RC goal skipped: Patrol points OFF");
        return;
    }

    m_targetWorld = worldPos;
    m_hasTarget = true;
    m_lastRobotStatusEvent.rcTargetX = worldPos.x();
    m_lastRobotStatusEvent.rcTargetY = worldPos.y();
    m_lastRobotStatusEvent.rcTargetZ = 0.0;
    if (m_overlay) {
        m_overlay->setGoalMarkerVisible(m_patrolPointsEnabled);
        if (m_patrolPointsEnabled) m_overlay->setGoalMarker(worldPos);
    }
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->setVisible(false);
        rcMap->setGoalMarkerVisible(false);
        updateCentralLayerOrder();
    }

    if (!m_pub) {
        appendLog("RC goal publish skipped: publisher is null");
        return;
    }

    QJsonObject o;
    o["x"] = worldPos.x();
    o["y"] = worldPos.y();
    o["frame"] = "world";
    o["ts_ms"] = QDateTime::currentMSecsSinceEpoch();

    const QByteArray payload = QJsonDocument(o).toJson(QJsonDocument::Compact);
    const bool ok = m_pub->publishJson("wiserisk/rc/goal", payload, 1, false);
    if (ok) {
        appendLog(QString("RC goal published: x=%1 y=%2")
                      .arg(worldPos.x(), 0, 'f', 3)
                      .arg(worldPos.y(), 0, 'f', 3));
        if (m_robotStatusWindow) {
            m_robotStatusWindow->setRobotStatus(m_lastRobotStatusEvent);
        }
    } else {
        appendLog("RC goal publish failed");
    }
}

void MainWindow::onCentralGoalClicked(const QPointF& widgetPos)
{
    if (!m_cameraGoalMode) return;
    if (!ui || !ui->centralOverlayHost) return;
    if (!m_hasHomography) {
        appendLog("Camera click ignored: homography unavailable");
        return;
    }
    QPointF imgPt = widgetPos;
    QTransform hImgToWorld;
    bool ok = false;

    if (m_homographyFromMqttImage && ui->labelCentralView) {
        if (!ui->labelCentralView->widgetToImagePoint(widgetPos, imgPt)) {
            appendLog("Camera click ignored: outside video area");
            return;
        }
        hImgToWorld = m_hWorldToImg.inverted(&ok);
    } else {
        // Calibration-generated homography is in overlay widget coordinates.
        const QTransform effectiveWorldToImg = currentOverlayHomography();
        hImgToWorld = effectiveWorldToImg.inverted(&ok);
    }
    if (!ok) {
        appendLog("Camera click ignored: H inversion failed");
        return;
    }

    const QPointF worldPos = hImgToWorld.map(imgPt);
    if (!std::isfinite(worldPos.x()) || !std::isfinite(worldPos.y())) {
        appendLog("Camera click ignored: non-finite world coordinate");
        return;
    }

    // Warn on unstable projection results (outside calibrated map bounds by margin),
    // but keep processing so the user can still see and verify the clicked marker.
    if (m_calibWorldPts.size() == 4) {
        double minX = m_calibWorldPts[0].x();
        double maxX = m_calibWorldPts[0].x();
        double minY = m_calibWorldPts[0].y();
        double maxY = m_calibWorldPts[0].y();
        for (const auto& p : m_calibWorldPts) {
            minX = std::min(minX, p.x());
            maxX = std::max(maxX, p.x());
            minY = std::min(minY, p.y());
            maxY = std::max(maxY, p.y());
        }
        const double mx = (maxX - minX) * 0.25 + 0.2;
        const double my = (maxY - minY) * 0.25 + 0.2;
        if (worldPos.x() < minX - mx || worldPos.x() > maxX + mx ||
            worldPos.y() < minY - my || worldPos.y() > maxY + my) {
            appendLog(QString("Camera click warning: out-of-bounds world=(%1,%2)")
                          .arg(worldPos.x(), 0, 'f', 3)
                          .arg(worldPos.y(), 0, 'f', 3));
        }
    }

    appendLog(QString("Camera click img=(%1,%2) -> world=(%3,%4)")
                  .arg(imgPt.x(), 0, 'f', 1)
                  .arg(imgPt.y(), 0, 'f', 1)
                  .arg(worldPos.x(), 0, 'f', 3)
                  .arg(worldPos.y(), 0, 'f', 3));
    onMiniMapWorldClicked(worldPos);
}

void MainWindow::onStartCaptureCalib()
{
    if (!ui->centralOverlayHost || !ui->labelCentralView) {
        appendLog("Capture calibration failed: central widgets missing");
        return;
    }

    QImage frame = ui->labelCentralView->grab().toImage();
    if (frame.isNull()) {
        appendLog("Capture calibration failed: frame capture is null");
        return;
    }

    if (!m_captureOverlay) {
        m_captureOverlay = new CaptureCalibOverlay(ui->centralOverlayHost);
        connect(m_captureOverlay, &CaptureCalibOverlay::calibrationSaved,
                this, &MainWindow::applyCalibImagePoints);
        connect(m_captureOverlay, &CaptureCalibOverlay::closed, this, [this]{
            if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
                rcMap->setVisible(false);
                rcMap->setGoalMarkerVisible(false);
            }
            if (m_overlay) {
                m_overlay->setGoalMarkerVisible(m_patrolPointsEnabled && m_hasTarget);
                if (m_patrolPointsEnabled && m_hasTarget) m_overlay->setGoalMarker(m_targetWorld);
            }
            updateCentralLayerOrder();
            appendLog("Capture calibration closed");
        });
    }

    auto pointsInFrame = [&frame](const QVector<QPointF>& pts) -> bool {
        if (pts.size() != 4) return false;
        const double w = frame.width();
        const double h = frame.height();
        int inside = 0;
        for (const auto& p : pts) {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y())) return false;
            if (p.x() >= 0.0 && p.x() <= w && p.y() >= 0.0 && p.y() <= h) inside++;
        }
        // Allow a small amount of drift but reject mostly off-screen sets.
        return inside >= 3;
    };

    QVector<QPointF> seedPts = m_calibImgPts;
    if (!pointsInFrame(seedPts) && m_hasHomography && m_calibWorldPts.size() == 4) {
        seedPts.clear();
        for (const auto& w : m_calibWorldPts) {
            seedPts.push_back(m_hWorldToImg.map(w));
        }
    }
    if (!pointsInFrame(seedPts)) {
        // Final fallback: centered rectangle so user always sees and edits points.
        const double w = frame.width();
        const double h = frame.height();
        seedPts = {
            QPointF(w * 0.30, h * 0.70), // 10
            QPointF(w * 0.70, h * 0.70), // 11
            QPointF(w * 0.30, h * 0.30), // 12
            QPointF(w * 0.70, h * 0.30)  // 13
        };
        appendLog("Capture seed fallback: using centered rectangle");
    }

    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) rcMap->hide();
    m_captureOverlay->setGeometry(ui->centralOverlayHost->contentsRect());
    m_captureOverlay->setImage(frame);
    m_captureOverlay->setInitialPoints(seedPts);
    QVector<QPointF> worldPoly = m_calibWorldPts;
    if (!worldPoly.isEmpty()) worldPoly.push_back(worldPoly.front());
    m_captureOverlay->setWorldPolygon(worldPoly);
    m_captureOverlay->setHomography(m_hWorldToImg, m_hasHomography);
    m_captureOverlay->setMapOverlayVisible(m_mapOverlayEnabled);
    m_captureOverlay->show();
    updateCentralLayerOrder();
    appendLog("Capture calibration opened");
}

void MainWindow::applyCalibImagePoints(const QVector<QPointF>& imgPts)
{
    if (imgPts.size() != 4) {
        appendLog("Capture calibration save failed: need 4 points");
        return;
    }

    m_calibImgPts = imgPts;
    rebuildHomographyFromCalib();
    // Saved capture calibration becomes the authoritative plane transform.
    m_homographyLocked = true;
    // After save, allow immediate map overlay preview/toggle with btnMap.
    m_mapOverlayEnabled = true;
    if (m_overlay) m_overlay->setMapVisible(true);
    rebuildPrimaryNavList();
    saveUiSettings();
    saveOverlayTweak();
    publishHomographyImageToWorld();

    if (m_hasHomography && m_calibWorldPts.size() == 4 && m_calibImgPts.size() == 4) {
        double sum = 0.0;
        for (int i = 0; i < 4; ++i) {
            const QPointF p = m_hWorldToImg.map(m_calibWorldPts[i]);
            const double d = QLineF(p, m_calibImgPts[i]).length();
            sum += d * d;
        }
        const double rmse = std::sqrt(sum / 4.0);
        appendLog(QString("Calibration RMSE(px) = %1").arg(rmse, 0, 'f', 2));
    }

    if (m_captureOverlay) {
        m_captureOverlay->hide();
    }
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->setVisible(false);
        rcMap->setGoalMarkerVisible(false);
    }
    if (m_overlay) {
        m_overlay->setGoalMarkerVisible(m_patrolPointsEnabled && m_hasTarget);
        if (m_patrolPointsEnabled && m_hasTarget) m_overlay->setGoalMarker(m_targetWorld);
    }
    updateCentralLayerOrder();
    appendLog("Capture calibration saved and applied");
}

void MainWindow::syncCentralOverlayGeometry()
{
    if (!ui || !ui->centralOverlayHost) return;

    const QRect r = ui->centralOverlayHost->contentsRect();
    if (ui->labelCentralView) ui->labelCentralView->setGeometry(r);
    if (ui->labelCentralMap) ui->labelCentralMap->setGeometry(r);
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->setGeometry(r);
    }
    if (auto* capMap = findChild<MiniMapWidget*>("labelCentralCapture")) {
        capMap->setGeometry(r);
    }
    if (m_overlay) {
        m_overlay->setGeometry(r);
        applyOverlayTransform();
    }
    if (m_captureOverlay) {
        m_captureOverlay->setGeometry(r);
    }
    updatePoseOverlayGeometry();
}

void MainWindow::updatePoseOverlayGeometry()
{
    if (!m_poseOverlay || !ui || !ui->centralOverlayHost) return;

    m_poseOverlay->setGeometry(ui->centralOverlayHost->contentsRect());

    if (ui->labelCentralView) {
        m_poseOverlay->setVideoGeometry(
            ui->labelCentralView->videoDisplayRect(),
            ui->labelCentralView->currentFrameSize()
        );
    } else {
        m_poseOverlay->setVideoGeometry(ui->centralOverlayHost->contentsRect());
    }

    m_poseOverlay->raise();

    if (m_humanBoxOverlay) {
        m_humanBoxOverlay->setGeometry(ui->centralOverlayHost->contentsRect());
        if (ui->labelCentralView) {
            QSize sourceSize = ui->labelCentralView->currentFrameSize();
            if (!sourceSize.isValid() ||
                sourceSize.width() < m_humanBoxSourceSize.width() ||
                sourceSize.height() < m_humanBoxSourceSize.height()) {
                sourceSize = m_humanBoxSourceSize;
            }
            m_humanBoxOverlay->setVideoGeometry(
                ui->labelCentralView->videoDisplayRect(),
                sourceSize
            );
        } else {
            m_humanBoxOverlay->setVideoGeometry(ui->centralOverlayHost->contentsRect(), m_humanBoxSourceSize);
        }
        m_humanBoxOverlay->raise();
    }
}

void MainWindow::updateCentralLayerOrder()
{
    if (m_overlay) m_overlay->raise();
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->setAttribute(Qt::WA_TransparentForMouseEvents, m_cameraGoalMode);
        if (rcMap->isVisible()) rcMap->raise();
    }
    if (auto* capMap = findChild<MiniMapWidget*>("labelCentralCapture")) {
        if (capMap->isVisible()) capMap->raise();
    }
    if (m_captureOverlay && m_captureOverlay->isVisible()) {
        m_captureOverlay->raise();
    }
    if (m_poseOverlay) {
        m_poseOverlay->raise();
    }
    if (m_humanBoxOverlay) {
        m_humanBoxOverlay->raise();
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (!event) {
        QMainWindow::keyPressEvent(event);
        return;
    }

    if (event->key() == Qt::Key_F11) {
        if (isFullScreen()) {
            showNormal();
            appendLog("Window mode: normal");
        } else {
            showFullScreen();
            appendLog("Window mode: fullscreen");
        }
        event->accept();
        return;
    }
    if (event->key() == Qt::Key_Escape && isFullScreen()) {
        showNormal();
        appendLog("Window mode: normal");
        event->accept();
        return;
    }

    const bool ctrl = event->modifiers() & Qt::ControlModifier;

    if (ctrl && event->key() == Qt::Key_S) {
        saveOverlayTweak();
        appendLog(QString("Overlay tweak saved dx=%1 dy=%2 s=%3 r=%4")
                      .arg(m_tweakDx).arg(m_tweakDy).arg(m_tweakScale).arg(m_tweakRotDeg));
        event->accept();
        return;
    }
    if (ctrl && event->key() == Qt::Key_L) {
        loadOverlayTweak();
        applyOverlayTransform();
        appendLog(QString("Overlay tweak loaded dx=%1 dy=%2 s=%3 r=%4")
                      .arg(m_tweakDx).arg(m_tweakDy).arg(m_tweakScale).arg(m_tweakRotDeg));
        event->accept();
        return;
    }

    bool changed = true;
    switch (event->key()) {
    case Qt::Key_Left:  m_tweakDx -= 2.0; break;
    case Qt::Key_Right: m_tweakDx += 2.0; break;
    case Qt::Key_Up:    m_tweakDy -= 2.0; break;
    case Qt::Key_Down:  m_tweakDy += 2.0; break;
    case Qt::Key_Plus:
    case Qt::Key_Equal: m_tweakScale *= 1.01; break;
    case Qt::Key_Minus: m_tweakScale /= 1.01; break;
    case Qt::Key_BracketLeft:  m_tweakRotDeg -= 0.2; break;
    case Qt::Key_BracketRight: m_tweakRotDeg += 0.2; break;
    case Qt::Key_0:
        m_tweakDx = 0.0;
        m_tweakDy = 0.0;
        m_tweakScale = 1.0;
        m_tweakRotDeg = 0.0;
        break;
    default:
        changed = false;
        break;
    }

    if (!changed) {
        QMainWindow::keyPressEvent(event);
        return;
    }

    if (m_tweakScale < 0.1) m_tweakScale = 0.1;
    if (m_tweakScale > 10.0) m_tweakScale = 10.0;

    applyOverlayTransform();
    appendLog(QString("Overlay tweak dx=%1 dy=%2 s=%3 r=%4")
                  .arg(m_tweakDx).arg(m_tweakDy).arg(m_tweakScale).arg(m_tweakRotDeg));
    event->accept();
}

//----- text color -----
void MainWindow::initEventStyleMap()
{
    m_eventColor["IvaArea"] = QColor(255, 50, 50);
    m_eventColor["SlipAndFallDetection"] = QColor(220, 60, 60);
    m_eventColor["ObjectCounterAlarm"] = QColor(230, 140, 40);
    m_eventColor["presence"] = QColor(255, 90, 90);
}

bool MainWindow::isDangerEventTopic(const QString& topic) const
{
    if (topic == "SlipAndFallDetection") return true;
    if (topic == "IvaArea") return true;
    return false;
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    QMainWindow::resizeEvent(e);
    updateCentralAspectRatio();
    updateSideVideoAspectRatios();

    // Responsive UI scaling for fullscreen / large-window modes.
    const double sx = static_cast<double>(width()) / 1578.0;
    const double sy = static_cast<double>(height()) / 738.0;
    double scale = std::min(sx, sy);
    if (scale < 0.85) scale = 0.85;
    if (scale > 1.60) scale = 1.60;

    if (isMaximized()) scale *= 1.28;
    const int btnH = std::max(20, static_cast<int>(std::round(20.0 * scale)));
    auto applyBtnHeight = [btnH](QPushButton* b) {
        if (!b) return;
        b->setMinimumHeight(btnH);
        b->setMaximumHeight(btnH);
    };
    applyBtnHeight(findChild<QPushButton*>("btnCapture"));
    applyBtnHeight(findChild<QPushButton*>("btnMap"));
    applyBtnHeight(findChild<QPushButton*>("ruviewbtn"));
    applyBtnHeight(findChild<QPushButton*>("btnOpenAllCctv"));
    const auto tankControlButtons = findChildren<QPushButton*>();
    for (QPushButton* button : tankControlButtons) {
        if (button && button->property("tankControlButton").toBool()) {
            const int tankBtnH = std::max(30, btnH + 10);
            button->setMinimumHeight(tankBtnH);
            button->setMaximumHeight(tankBtnH);
        }
    }
    if (m_tankHeaderHost) {
        m_tankHeaderHost->setMinimumHeight(btnH + 4);
        m_tankHeaderHost->setMaximumHeight(btnH + 4);
    }

    QFont base = font();
    base.setPointSize(std::max(9, static_cast<int>(std::round(10.0 * scale))));
    setFont(base);

    if (auto* nameBar = findChild<QWidget*>("platformNameBar")) {
        const int barH = isMaximized() ? 38 : 30;
        nameBar->setMinimumHeight(barH);
        nameBar->setMaximumHeight(barH);
    }
    if (auto* nameLabel = findChild<QLabel*>("platformNameLabel")) {
        QFont f = nameLabel->font();
        f.setPointSize(isMaximized() ? 15 : 12);
        f.setBold(true);
        nameLabel->setFont(f);
    }

    if (ui && ui->textMsgLog) {
        int monoPt = std::max(8, static_cast<int>(std::round(9.0 * scale)));
        if (isMaximized()) monoPt = (height() >= 900) ? 11 : 10;
        QFont mono("Consolas", monoPt);
        ui->textMsgLog->setFont(mono);
    }

    syncCentralOverlayGeometry();
    updateCentralLayerOrder();
    if (m_primaryNavList) {
        rebuildPrimaryNavList();
        m_primaryNavList->doItemsLayout();
        m_primaryNavList->viewport()->update();
    }
}

void MainWindow::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);
    QTimer::singleShot(0, this, [this]{
        if (m_overlay && m_hasHomography) {
            applyOverlayTransform();
        }
        rebuildPrimaryNavList();
        if (m_primaryNavList) {
            m_primaryNavList->doItemsLayout();
            m_primaryNavList->viewport()->update();
        }
    });
}










void MainWindow::setupCustomTitleBar()
{
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint);

    m_titleBar = new TitleBarWidget(this);
    m_titleBar->setTitleText(windowTitle());
    m_titleBar->setMaximized(isMaximized());
    m_titleBar->setMaximizeVisible(false);

    setMenuWidget(m_titleBar);

    connect(this, &QWidget::windowTitleChanged,
            m_titleBar, &TitleBarWidget::setTitleText);
    connect(m_titleBar, &TitleBarWidget::minimizeRequested,
            this, &QWidget::showMinimized);
    connect(m_titleBar, &TitleBarWidget::closeRequested,
            this, &QWidget::close);
}

bool MainWindow::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
    if (FramelessHelper::handleNativeEvent(this, m_titleBar, eventType, message, result))
        return true;
    return QMainWindow::nativeEvent(eventType, message, result);
}

void MainWindow::changeEvent(QEvent* event)
{
    if (event && event->type() == QEvent::WindowStateChange) {
        if (m_titleBar) m_titleBar->setMaximized(isMaximized());
        QTimer::singleShot(0, this, [this]{
            applyMainSplitterDefaultSizes();
            updateCentralAspectRatio();
            updateSideVideoAspectRatios();
            syncCentralOverlayGeometry();
            updateCentralLayerOrder();
        });
    }
    QMainWindow::changeEvent(event);
}
