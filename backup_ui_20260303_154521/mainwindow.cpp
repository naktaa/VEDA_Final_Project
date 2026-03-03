#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ClipPopup.h"
#include "video_clip.h"
#include "AllCctvWindow.h"
#include "HomographyCalib.h"
#include "CctvOverlayWidget.h"
#include "MiniMapWidget.h"
#include "CaptureCalibOverlay.h"

#include "GstVideoWidget.h"

#include <QTimer>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <QLineF>
#include <QKeyEvent>
#include <QPushButton>
#include <QSettings>
#include <QStackedLayout>
#include <QFont>
#include <algorithm>
#include <cmath>

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
    // centralOverlayHost ?àÏóê?úÎßå Í≤πÏπòÍ∏?Íµ¨ÏÑ±
    auto* stack = new QStackedLayout(ui->centralOverlayHost);
    stack->setContentsMargins(0,0,0,0);
    stack->setStackingMode(QStackedLayout::StackAll); // ???µÏã¨: ??Î≥¥Ïù¥Í≤?Í≤πÏπ®

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
        appendLog(QString("Map overlay %1").arg(m_mapOverlayEnabled ? "ON" : "OFF"));
    });
    if (auto* btnRc = findChild<QPushButton*>("btnRC")) {
        btnRc->setCheckable(true);
        btnRc->setAutoExclusive(false);
        btnRc->setChecked(m_patrolPointsEnabled);
        connect(btnRc, &QPushButton::toggled, this, [this](bool on){
            m_patrolPointsEnabled = on;
            m_cameraGoalMode = on;
            if (m_overlay) m_overlay->setGoalClickMode(m_cameraGoalMode);
            if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
                rcMap->setVisible(true);
                rcMap->setGoalMarkerVisible(false);
            }
            if (m_overlay) {
                m_overlay->setGoalMarkerVisible(on && m_hasTarget);
                if (on && m_hasTarget) m_overlay->setGoalMarker(m_targetWorld);
            }

            if (on && m_pub) {
                QJsonObject o;
                o["type"] = "patrol_points";
                o["enabled"] = on ? 1 : 0;
                o["ts_ms"] = QDateTime::currentMSecsSinceEpoch();
                const QByteArray payload = QJsonDocument(o).toJson(QJsonDocument::Compact);
                const bool ok = m_pub->publishJson("wiserisk/rc/patrol_points", payload, 1, false);
                appendLog(ok
                              ? QString("Patrol points %1 (MQTT sent)").arg(on ? "ON" : "OFF")
                              : QString("Patrol points %1 (MQTT failed)").arg(on ? "ON" : "OFF"));
            } else if (on) {
                appendLog(QString("Patrol points %1 (publisher null)").arg(on ? "ON" : "OFF"));
            } else {
                appendLog("Patrol points OFF (MQTT not sent)");
            }

            syncCentralOverlayGeometry();
            updateCentralLayerOrder();
        });
    }
    if (auto* btnCapture = findChild<QPushButton*>("btnCapture")) {
        connect(btnCapture, &QPushButton::clicked, this, &MainWindow::onStartCaptureCalib);
    } else if (auto* btnCaptureAlt = findChild<QPushButton*>("btnMap_2")) {
        connect(btnCaptureAlt, &QPushButton::clicked, this, &MainWindow::onStartCaptureCalib);
    }
    if (auto* patrolStartBtn = findChild<QPushButton*>("patrolstartbtn")) {
        patrolStartBtn->setCheckable(true);
        patrolStartBtn->setAutoExclusive(false);
        patrolStartBtn->setChecked(false);

        connect(patrolStartBtn, &QPushButton::toggled, this, [this, patrolStartBtn](bool on){
            if (!m_pub) {
                appendLog(QString("Patrol %1 failed: publisher null").arg(on ? "start" : "stop"));
                return;
            }

            QJsonObject o;
            o["type"] = "patrol_control";
            o["command"] = on ? "start" : "stop";
            o["ts_ms"] = QDateTime::currentMSecsSinceEpoch();

            const QByteArray payload = QJsonDocument(o).toJson(QJsonDocument::Compact);
            const bool ok = m_pub->publishJson("wiserisk/rc/patrol/cmd", payload, 1, false);
            appendLog(ok
                          ? QString("Patrol %1 command sent").arg(on ? "start" : "stop")
                          : QString("Patrol %1 command failed").arg(on ? "start" : "stop"));
        });
    }

    // init
    initUiPlaceholders();
    initCentralStreamMap();
    initStreamMap();
    initWardTree();
    initEventStyleMap();
    initLocalMapFallback();
    loadOverlayTweak();

    m_centralPlayingUrl.clear();
    m_tankPlayingUrl.clear();
    m_patrolPlayingUrl.clear();

    // MQTT
    m_mqtt = new MqttSubscriber(nullptr); // parent Ï£ºÎ©¥ moveToThread?êÏÑú Î¨∏Ï†ú?¨ÏóàÏßÄ
    connect(m_mqtt, &MqttSubscriber::eventReceived,
            this, &MainWindow::onMqttEvent, Qt::QueuedConnection);
    connect(m_mqtt, &MqttSubscriber::logLine,
            this, &MainWindow::onMqttLogLine, Qt::QueuedConnection);
    connect(m_mqtt, &MqttSubscriber::homographyReceived,
            this, &MainWindow::onHomographyReceived, Qt::QueuedConnection);
    m_pub = new MqttPublisher(this);
    connect(m_pub, &MqttPublisher::logLine,
            this, &MainWindow::onMqttLogLine, Qt::QueuedConnection);

    // Central overlay (Í≤ΩÎ≥¥ ?§Î≤Ñ?àÏù¥)
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
            QVector<QPointF> poly = {
                m_calibWorldPts[0],
                m_calibWorldPts[1],
                m_calibWorldPts[2],
                m_calibWorldPts[3],
                m_calibWorldPts[0]
            };
            m_overlay->setNodes(ons);
            m_overlay->setPolyline(poly);
        }
        m_overlay->setMapVisible(m_mapOverlayEnabled);
        m_overlay->raise();
        m_overlay->show();
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
        // (?†ÌÉù) ÏßÄ???¥Î¶≠ ÎßâÍ≥† CCTV ?¥Î¶≠Îß?Î∞õÍ≤å
        // ui->labelCentralMap->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    } else {
        appendLog("labelCentralMap is null (Promote/objectName ?ïÏù∏)");
    }
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->setWorldSize(5.0, 5.0);
        rcMap->setAttribute(Qt::WA_TransparentForMouseEvents, false);
        rcMap->setDrawMapGeometry(false); // RC layer should show goal point only.
        connect(rcMap, &MiniMapWidget::worldClicked,
                this, &MainWindow::onMiniMapWorldClicked);
        rcMap->setGoalMarkerVisible(false); // Use CCTV overlay marker for visual consistency.
        rcMap->show();
    }
    if (auto* capMap = findChild<MiniMapWidget*>("labelCentralCapture")) {
        capMap->setWorldSize(5.0, 5.0);
        capMap->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        capMap->hide();
    }
    m_mapOverlayEnabled = false;

    // Î∏åÎ°úÏª??úÎ≤Ñ IPÎ°??ëÏÜç
    m_mqtt->start("192.168.100.10", 1883, "wiserisk/#");
    m_pub->start("192.168.100.10", 1883, "qt-main-goal-pub");

    // UI connect
    connect(ui->btnOpenAllCctv, &QPushButton::clicked,
            this, &MainWindow::onOpenAllCctv);

    connect(ui->listWardCctv, &QTreeWidget::itemClicked,
            this, &MainWindow::onWardItemClicked);

    connect(ui->listEvent, &QListWidget::itemClicked,
            this, &MainWindow::onEventClicked);

    // status
    const QString roleText =
        (m_role == UserRole::Executive) ? "Executive" : "ControlRoom";
    statusBar()->showMessage(
        QString("Login: %1 | Role: %2")
            .arg(m_userId.isEmpty() ? "Unknown" : m_userId, roleText)
        );
    appendLog("System started");

    // ??Ï§ëÏïô Í∏∞Î≥∏ Ï±ÑÎÑê (Central CCTV Î≤ÑÌäº/Î°úÏßÅÍ≥??ôÏùº)
    QTimer::singleShot(0, this, [this]{
        syncCentralOverlayGeometry();
        switchCentralChannel(1); // centralcctv Ï™?Î°úÏßÅ ?†Ï?/?ïÏã§???§Ìñâ
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
    if (ui->labelPatrolView)  ui->labelPatrolView->stopStream();

    delete ui;
}

// ---------------- init ----------------
void MainWindow::initUiPlaceholders()
{
    setStyleSheet(R"(
QMainWindow, QWidget#centralWidget {
    background-color: #6d6a64;
    color: #333333;
}
QGroupBox {
    border: 1px solid #988575;
    border-radius: 4px;
    margin-top: 10px;
    padding-top: 12px;
    background-color: #c2b8b0;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    color: #00E5FF;
    font-weight: 700;
}
QPushButton {
    background-color: #e3ddda;
    color: #333333;
    border: 1px solid #665950;
    border-radius: 3px;
    padding: 1px 8px;
    min-height: 20px;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #bcb2aa;
}
QPushButton:pressed {
    background-color: #b3a79e;
}
QPushButton:checked {
    background-color: #ada096;
    border-color: #665950;
    color: #333333;
}
QListWidget, QTreeWidget, QPlainTextEdit {
    background-color: #e3ddda;
    color: #6d6a64;
    border: 1px solid #988575;
    border-radius: 3px;
    selection-background-color: #988575;
    selection-color: #333333;
}

QTreeWidget {
    font-weight: 700;
}
QLabel {
    background-color: #6d6a64;
    color: #333333;
}
QStatusBar {
    background-color: #534842;
    color: #c2b8b0;
}
)");

    if (ui->labelCentralView) ui->labelCentralView->setStyleSheet("background-color:black;");
    if (ui->labelTankView)    ui->labelTankView->setStyleSheet("background-color:black;");
    if (ui->labelPatrolView)  ui->labelPatrolView->setStyleSheet("background-color:black;");

    ui->textMsgLog->setReadOnly(true);
    ui->textMsgLog->setMaximumBlockCount(1000);
    ui->textMsgLog->setFont(QFont("Consolas", 10));

    auto styleSmallToggle = [](QPushButton* b) {
        if (!b) return;        b->setMinimumHeight(20);
        b->setMaximumHeight(20);
        b->setCheckable(true);
        b->setAutoExclusive(false);
    };
    styleSmallToggle(findChild<QPushButton*>("btnCapture"));
    styleSmallToggle(findChild<QPushButton*>("btnMap"));
    styleSmallToggle(findChild<QPushButton*>("btnRC"));

    if (auto* patrolStartBtn = findChild<QPushButton*>("patrolstartbtn")) {        patrolStartBtn->setMinimumHeight(20);
        patrolStartBtn->setMaximumHeight(20);
    }
    if (auto* openAllBtn = findChild<QPushButton*>("btnOpenAllCctv")) {        openAllBtn->setMinimumHeight(20);
        openAllBtn->setMaximumHeight(20);
    }
}

void MainWindow::initCentralStreamMap()
{
    // Ï§ëÏïô Ï±ÑÎÑê Î≤ÑÌäº(centralcctv)??
    m_centralRtsp[1] = "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
}

void MainWindow::initStreamMap()
{
    // ???∏Î¶¨?êÏÑú ?§Ï†úÎ°?Ï°¥Ïû¨?òÎäî ?§Ìä∏Î¶ºÎßå ?±Î°ù (Îª•Ïπ¥??ÎØ∏Îì±Î°?
    m_wardStreams.clear();

    // Main1 -> CentralView (?§Ï†ú)
    //m_wardStreams["Main1"] = { "rtsp://192.168.100.8:8554/live", QString() };

    // T1 -> TankView (?§Ï†ú)
    m_wardStreams["T1"]    = { "rtsp://192.168.100.8:8554/live", QString() };

    // P1 -> PatrolView (?§Ï†ú)
    m_wardStreams["P1"]    = { "rtsp://192.168.100.29:8554/live", QString() };

    // Main2/Main3/T2/T3/P2/P3 ???ºÎ????±Î°ù?òÏ? ?äÏùå (= ?îÎ?/Îª•Ïπ¥)
}

void MainWindow::initWardTree()
{
    ui->listWardCctv->clear();

    ui->listWardCctv->setColumnCount(1);
    ui->listWardCctv->setHeaderHidden(true);
    ui->listWardCctv->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui->listWardCctv->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    ui->listWardCctv->setRootIsDecorated(true);
    ui->listWardCctv->setIndentation(20);

    auto *restricted = new QTreeWidgetItem(ui->listWardCctv);
    restricted->setText(0, "Restricted Zone");
    (new QTreeWidgetItem(restricted))->setText(0, "Main1");
    (new QTreeWidgetItem(restricted))->setText(0, "Main2");
    (new QTreeWidgetItem(restricted))->setText(0, "Main3");

    auto *tank = new QTreeWidgetItem(ui->listWardCctv);
    tank->setText(0, "Tank Unit");
    (new QTreeWidgetItem(tank))->setText(0, "T1");
    (new QTreeWidgetItem(tank))->setText(0, "T2");
    (new QTreeWidgetItem(tank))->setText(0, "T3");

    auto *recon = new QTreeWidgetItem(ui->listWardCctv);
    recon->setText(0, "Recon Unit");
    (new QTreeWidgetItem(recon))->setText(0, "P1");
    (new QTreeWidgetItem(recon))->setText(0, "P2");
    (new QTreeWidgetItem(recon))->setText(0, "P3");

    // ?ÅÏúÑ ??™© ?†ÌÉù Î∂àÍ?
    restricted->setFlags(restricted->flags() & ~Qt::ItemIsSelectable);
    tank->setFlags(tank->flags() & ~Qt::ItemIsSelectable);
    recon->setFlags(recon->flags() & ~Qt::ItemIsSelectable);

    ui->listWardCctv->expandAll();

    // Ï¥àÍ∏∞ ?†ÌÉù
    auto *firstLeaf = restricted->child(0);
    if (firstLeaf) ui->listWardCctv->setCurrentItem(firstLeaf);
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

    ui->labelCentralView->stopStream();
    ui->labelCentralView->startStream(url);
    updateCentralLayerOrder();

    m_centralPlayingUrl = url;
    appendLog(QString("Central switch -> CH%1").arg(ch));
}

// ---------------- tree click routing ----------------
void MainWindow::onWardItemClicked(QTreeWidgetItem* item, int column)
{
    Q_UNUSED(column);
    if (!item) return;

    // Í∑∏Î£π?¥Î©¥ ?†Í?Îß?
    if (item->childCount() > 0) {
        item->setExpanded(!item->isExpanded());
        return;
    }

    const QString key = item->text(0).trimmed();

    // ???îÎ?/Îª•Ïπ¥Î©??ÑÎ¨¥ Í≤ÉÎèÑ ????
    if (!m_wardStreams.contains(key)) {
        appendLog(QString("Dummy item (no stream): %1").arg(key));
        return;
    }

    // ????prefixÎ°?Î∑??ºÏö∞??
    if (key.startsWith("Main")) {
        switchCentralStreamTo(key);
    } else if (key.startsWith("T")) {
        switchTankStreamTo(key);
    } else if (key.startsWith("P")) {
        switchPatrolStreamTo(key);
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
    const QString url = p.rawRtsp.trimmed(); // rawÎß??¨Ïö©

    if (url.isEmpty()) return;
    if (m_centralPlayingUrl == url) {
        appendLog(QString("Central same url -> skip (%1)").arg(key));
        return;
    }

    ui->labelCentralView->stopStream();
    ui->labelCentralView->startStream(url);
    m_centralPlayingUrl = url;

    appendLog(QString("Central switch -> %1").arg(key));
}

void MainWindow::switchTankStreamTo(const QString& key)
{
    if (!ui->labelTankView) {
        appendLog("labelTankView not found (Promote/objectName ?ïÏù∏)");
        return;
    }

    const auto p = m_wardStreams.value(key);
    const QString url = p.rawRtsp.trimmed();

    if (url.isEmpty()) return;
    if (m_tankPlayingUrl == url) {
        appendLog(QString("Tank same url -> skip (%1)").arg(key));
        return;
    }

    ui->labelTankView->stopStream();
    ui->labelTankView->startStream(url);
    m_tankPlayingUrl = url;

    appendLog(QString("Tank switch -> %1").arg(key));
}

void MainWindow::switchPatrolStreamTo(const QString& key)
{
    if (!ui->labelPatrolView) {
        appendLog("labelPatrolView not found (Promote/objectName ?ïÏù∏)");
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

    // ===== MQTT ?¥Î≤§???¥Î¶≠ ?¨ÏÉù =====
    if (ev.source == "mqtt") {
        const QString clipUrl  = item->data(Qt::UserRole + 1).toString();
        int clipSec            = item->data(Qt::UserRole + 2).toInt();
        const QString topic    = item->data(Qt::UserRole + 3).toString();
        const QString cam      = item->data(Qt::UserRole + 4).toString();
        const QString utcShort = item->data(Qt::UserRole + 5).toString();

        if (!isDangerEventTopic(topic)) {
            appendLog(QString("MQTT clicked (not danger) topic=%1").arg(topic));
            return;
        }

        if (clipUrl.isEmpty()) {
            appendLog("MQTT danger event but clip_url is empty");
            return;
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

        appendLog(QString("Play clip popup: %1").arg(clipUrl));
        return;
    }

    // ===== Í∏∞Ï°¥ ward ?¥Î≤§??Ï≤òÎ¶¨ ?†Ï? =====
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

    appendLog(QString("Open clip window | src=%1 | id=%2").arg(ev.source, ev.id));
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

    // AllCCTV???§Ìä∏Î¶?Îß?Ï£ºÏûÖ
    QMap<int, QString> rtsp;
    rtsp[1] = m_centralRtsp.value(1);

    QMap<int, QString> titles;
    titles[1] = "Center CH1";

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
}

QString MainWindow::clipTitle(const EventItem& ev) const
{
    const QString roleText = (m_role == UserRole::Executive) ? "Executive" : "ControlRoom";
    const QString src = ev.source;
    const QString ward = ev.wardName.isEmpty() ? "" : QString(" | %1").arg(ev.wardName);
    return QString("Clip | %1 | %2%3 | %4").arg(roleText, src, ward, ev.id);
}

// ------------- MQTT service ------------------
void MainWindow::onMqttLogLine(const QString& s)
{
    appendLog(QString("[MQTT] %1").arg(s));
}

void MainWindow::onMqttEvent(const MqttEvent& ev)
{
    if (ev.topic == "MotionAlarm" || ev.topic == "MotionDetection")
        return;

    if (!ev.state) return;

    QString utcShort = ev.utc;
    int dot = utcShort.indexOf('.');
    if (dot > 0) utcShort = utcShort.left(dot);
    utcShort.replace("Z", "");
    utcShort.replace("T", " ");

    QString src = ev.src.isEmpty() ? "unknown" : ev.src;

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

    QString firstLine = QString("[ %1 | %2 | %3 ]").arg(utcShort, ev.cam, src);

    QStringList second;
    second << ev.topic;
    if (!ev.rule.isEmpty())   second << ev.rule;
    if (!ev.action.isEmpty()) second << ev.action;

    QString line = firstLine + "\n " + second.join(" | ");

    auto* lw = new QListWidgetItem(line);
    lw->setData(Qt::UserRole, idx);

    lw->setData(Qt::UserRole + 1, ev.clipUrl);
    lw->setData(Qt::UserRole + 2, ev.clipSec > 0 ? ev.clipSec : 5);
    lw->setData(Qt::UserRole + 3, ev.topic);
    lw->setData(Qt::UserRole + 4, ev.cam);
    lw->setData(Qt::UserRole + 5, utcShort);
    lw->setData(Qt::UserRole + 6, src);

    auto it = m_eventColor.find(ev.topic);
    if (it != m_eventColor.end()) {
        lw->setForeground(QBrush(it.value()));
    }

    ui->listEvent->addItem(lw);
    ui->listEvent->scrollToBottom();

    if (isDangerEventTopic(ev.topic)) {
        if (ev.clipUrl.isEmpty()) {
            appendLog(QString("[DANGER] clip_url empty: %1 | %2").arg(ev.cam, ev.topic));
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
        appendLog("Homography ignored (locked after first apply)");
        return;
    }

    m_hWorldToImg = hWorldToImg;
    m_hasHomography = true;
    m_homographyLocked = true; // apply once, ignore repeated publish
    if (m_captureOverlay) m_captureOverlay->setHomography(m_hWorldToImg, true);
    if (m_calibImgPts.size() != m_calibWorldPts.size()) {
        m_calibImgPts.clear();
        m_calibImgPts.reserve(m_calibWorldPts.size());
        for (const auto& w : m_calibWorldPts)
            m_calibImgPts.push_back(m_hWorldToImg.map(w));
    }
    applyOverlayTransform();
    appendLog("Homography updated from MQTT (H_img2world, locked)");
}

void MainWindow::initLocalMapFallback()
{
    if (!ui->labelCentralMap) return;

    // User-provided world coordinates (IDs: 10 -> 11 -> 12 -> 13)
    const QVector<MapNode> nodes = {
        {"10", QPointF(0.0, 0.0)},
        {"11", QPointF(0.94, 0.0)},
        {"12", QPointF(0.94, 5.17)},
        {"13", QPointF(0.0, 5.17)}
    };
    const QVector<MapEdge> edges = {
        {"10", "11"},
        {"11", "12"},
        {"12", "13"},
        {"13", "10"}
    };
    const QVector<QPointF> polyline = {
        QPointF(0.0, 0.0),
        QPointF(0.94, 0.0),
        QPointF(0.94, 5.17),
        QPointF(0.0, 5.17),
        QPointF(0.0, 0.0)
    };
    m_calibWorldPts = {
        QPointF(0.0, 0.0),
        QPointF(0.94, 0.0),
        QPointF(0.94, 5.17),
        QPointF(0.0, 5.17)
    };

    const double worldW = 1.2;
    const double worldH = 5.5;
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
        m_overlay->setPolyline(polyline);
        m_overlay->setMapVisible(m_mapOverlayEnabled);
    }

    appendLog("Local map fallback loaded (10/11/12/13)");
}

void MainWindow::applyOverlayTransform()
{
    if (!m_overlay || !m_hasHomography) return;

    QTransform tweak;
    tweak.translate(m_tweakDx, m_tweakDy);

    const double cx = m_overlay->width() * 0.5;
    const double cy = m_overlay->height() * 0.5;
    tweak.translate(cx, cy);
    tweak.rotate(m_tweakRotDeg);
    tweak.scale(m_tweakScale, m_tweakScale);
    tweak.translate(-cx, -cy);

    m_overlay->setHomography(tweak * m_hWorldToImg);
}

void MainWindow::saveOverlayTweak() const
{
    QSettings s("overlay_calib.ini", QSettings::IniFormat);
    s.setValue("overlay/dx", m_tweakDx);
    s.setValue("overlay/dy", m_tweakDy);
    s.setValue("overlay/scale", m_tweakScale);
    s.setValue("overlay/rot_deg", m_tweakRotDeg);
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
    if (m_captureOverlay) m_captureOverlay->setHomography(m_hWorldToImg, true);
    applyOverlayTransform();
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
        QVector<QPointF> poly = {
            m_calibWorldPts[0],
            m_calibWorldPts[1],
            m_calibWorldPts[2],
            m_calibWorldPts[3],
            m_calibWorldPts[0]
        };
        m_overlay->setNodes(ons);
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
    if (m_overlay) {
        m_overlay->setGoalMarkerVisible(m_patrolPointsEnabled);
        if (m_patrolPointsEnabled) m_overlay->setGoalMarker(worldPos);
    }
    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) {
        rcMap->show();
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
    // Homography is calibrated in central overlay widget coordinates.
    // Use the same coordinate system for click inverse projection.
    const QPointF imgPt = widgetPos;

    // Use the same effective transform used by overlay rendering (tweak * H_world2img).
    QTransform effectiveWorldToImg = m_hWorldToImg;
    if (m_overlay) {
        QTransform tweak;
        tweak.translate(m_tweakDx, m_tweakDy);
        const double cx = m_overlay->width() * 0.5;
        const double cy = m_overlay->height() * 0.5;
        tweak.translate(cx, cy);
        tweak.rotate(m_tweakRotDeg);
        tweak.scale(m_tweakScale, m_tweakScale);
        tweak.translate(-cx, -cy);
        effectiveWorldToImg = tweak * m_hWorldToImg;
    }

    bool ok = false;
    const QTransform hImgToWorld = effectiveWorldToImg.inverted(&ok);
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
                rcMap->setVisible(true);
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
            QPointF(w * 0.30, h * 0.30), // 10
            QPointF(w * 0.70, h * 0.30), // 11
            QPointF(w * 0.70, h * 0.70), // 12
            QPointF(w * 0.30, h * 0.70)  // 13
        };
        appendLog("Capture seed fallback: using centered rectangle");
    }

    if (auto* rcMap = findChild<MiniMapWidget*>("labelCentralRC")) rcMap->hide();
    m_captureOverlay->setGeometry(ui->centralOverlayHost->rect());
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
    saveOverlayTweak();

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
        rcMap->setVisible(true);
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

    const QRect r = ui->centralOverlayHost->rect();
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

    // Responsive UI scaling for fullscreen / large-window modes.
    const double sx = static_cast<double>(width()) / 1578.0;
    const double sy = static_cast<double>(height()) / 738.0;
    double scale = std::min(sx, sy);
    if (scale < 0.85) scale = 0.85;
    if (scale > 1.60) scale = 1.60;

    const int btnH = std::max(20, static_cast<int>(std::round(20.0 * scale)));
    auto applyBtnHeight = [btnH](QPushButton* b) {
        if (!b) return;
        b->setMinimumHeight(btnH);
        b->setMaximumHeight(btnH);
    };
    applyBtnHeight(findChild<QPushButton*>("btnCapture"));
    applyBtnHeight(findChild<QPushButton*>("btnMap"));
    applyBtnHeight(findChild<QPushButton*>("btnRC"));
    applyBtnHeight(findChild<QPushButton*>("patrolstartbtn"));
    applyBtnHeight(findChild<QPushButton*>("btnOpenAllCctv"));

    QFont base = font();
    base.setPointSize(std::max(9, static_cast<int>(std::round(10.0 * scale))));
    setFont(base);
    if (ui && ui->textMsgLog) {
        QFont mono("Consolas", std::max(9, static_cast<int>(std::round(10.0 * scale))));
        ui->textMsgLog->setFont(mono);
    }

    syncCentralOverlayGeometry();
    updateCentralLayerOrder();
}









