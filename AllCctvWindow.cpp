#include "AllCctvWindow.h"
#include "ui_AllCctvWindow.h"
#include "CctvZoomWindow.h"
#include "GstVideoWidget.h"   // ???�일�??�?�문???�제?� ?�일?�게!
#include "FullscreenViewer.h"
#include "TitleBarWidget.h"
#include "FramelessHelper.h"
#include <QEvent>
#include <QMouseEvent>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QResizeEvent>
#include <QShowEvent>
#include <QDebug>
#include <QTimer>

#include <QMetaObject>
#include <QWidget>
#include <QRect>

// ---------------- util ----------------

static void clearGridLayoutOnly(QGridLayout* grid)
{
    if (!grid) return;
    while (grid->count() > 0) {
        QLayoutItem* item = grid->takeAt(0);
        delete item; // widget?� ??�� ????
    }
}

static QVBoxLayout* ensureFrameVBoxLayout(QFrame* frame)
{
    if (!frame) return nullptr;
    if (auto* existing = qobject_cast<QVBoxLayout*>(frame->layout()))
        return existing;

    auto* box = new QVBoxLayout(frame);
    box->setContentsMargins(0, 0, 0, 0);
    box->setSpacing(0);
    frame->setLayout(box);
    return box;
}

// ---------------- ctor/dtor ----------------

AllCctvWindow::AllCctvWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::AllCctvWindow)
{
    ui->setupUi(this);
    setupCustomTitleBar();

    setStyleSheet(R"(
QMainWindow, QWidget#centralWidget {
    background-color: #20293d;
    color: #dbe7ff;
}
QWidget#topBar, QWidget#gridArea {
    background-color: #28324a;
}
QWidget#platformNameBar {
    background-color: #2b3550;
    border: 1px solid #384764;
    border-radius: 6px;
}
QLabel#platformNameLabel {
    background: transparent;
    color: #f6fbff;
    font-weight: 800;
    letter-spacing: 0.5px;
}
QLabel#labelTitle {
    color: #f7fbff;
    font-weight: 800;
    font-size: 15px;
}
QComboBox, QPushButton {
    background-color: #384461;
    color: #eef5ff;
    border: 1px solid #4c628b;
    border-radius: 5px;
    min-height: 32px;
    padding: 0px 10px;
}
QComboBox {
    padding-top: 0px;
    padding-bottom: 0px;
}
QComboBox::drop-down {
    width: 24px;
    border: none;
    subcontrol-origin: padding;
    subcontrol-position: top right;
}
QComboBox::down-arrow {
    margin-top: 0px;
}
QComboBox#comboGrid {
    min-height: 26px;
    max-height: 26px;
    padding: 0px 8px;
    margin: 0px;
}
QComboBox#comboGrid::drop-down {
    width: 22px;
}
QPushButton:hover, QComboBox:hover {
    background-color: #445274;
}
QPushButton:pressed {
    background-color: #2d3854;
}
QComboBox QAbstractItemView {
    background-color: #2a344d;
    color: #eef5ff;
    border: 1px solid #4c628b;
    selection-background-color: #2dbcd0;
}
QFrame {
    background-color: #111722;
    border: 1px solid #334562;
    border-radius: 8px;
}
QLabel {
    background-color: transparent;
    color: #dbe7ff;
    font-size: 12px;
    border: none;
}
)");

    ensureRootLayout();

    connect(ui->comboGrid,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &AllCctvWindow::onGridChanged);

    bindUiObjects();

    // ??반드??16?�로 고정
    m_playingUrl.resize(16);
    for (int i = 0; i < 16; ++i) m_playingUrl[i].clear();

    applyGridModeFromCombo();
}

AllCctvWindow::~AllCctvWindow()
{
    closeAllStreams();
    delete ui;
}


// ---------------- layout fix ----------------

void AllCctvWindow::ensureRootLayout()
{
    QWidget* cw = ui->centralWidget;
    if (!cw || !ui->topBar || !ui->gridArea) return;

    if (QLayout* old = cw->layout()) {
        while (old->count() > 0) old->takeAt(0);
        delete old;
    }

    auto* root = new QVBoxLayout(cw);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    ui->topBar->setParent(cw);
    ui->gridArea->setParent(cw);

    auto* nameBar = new QWidget(cw);
    nameBar->setObjectName("platformNameBar");
    nameBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    nameBar->setMinimumHeight(26);
    nameBar->setMaximumHeight(26);
    auto* nameLayout = new QHBoxLayout(nameBar);
    nameLayout->setContentsMargins(8, 2, 8, 2);
    nameLayout->setSpacing(0);
    auto* nameLabel = new QLabel("All CCTV", nameBar);
    nameLabel->setObjectName("platformNameLabel");
    nameLabel->setAlignment(Qt::AlignCenter);
    QFont nameFont = nameLabel->font();
    nameFont.setPointSize(11);
    nameFont.setBold(true);
    nameLabel->setFont(nameFont);
    nameLayout->addWidget(nameLabel, 1, Qt::AlignCenter);

    root->addWidget(nameBar);
    root->addWidget(ui->topBar);
    root->addWidget(ui->gridArea);

    root->setStretch(0, 0);
    root->setStretch(1, 0);
    root->setStretch(2, 1);

    ui->topBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ui->topBar->setMinimumHeight(44);
    ui->topBar->setMaximumHeight(44);
    ui->gridArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    if (QLayout* topOld = ui->topBar->layout()) {
        while (topOld->count() > 0) topOld->takeAt(0);
        delete topOld;
    }
    if (ui->horizontalLayoutWidget) {
        ui->horizontalLayoutWidget->setParent(ui->topBar);
        auto* topLayout = new QHBoxLayout(ui->topBar);
        topLayout->setContentsMargins(12, 8, 12, 8);
        topLayout->setSpacing(0);
        topLayout->addWidget(ui->horizontalLayoutWidget, 1);
    }

    if (QLayout* gridOld = ui->gridArea->layout()) {
        while (gridOld->count() > 0) gridOld->takeAt(0);
        delete gridOld;
    }
    if (ui->gridLayoutWidget) {
        ui->gridLayoutWidget->setParent(ui->gridArea);
        auto* gridAreaLayout = new QVBoxLayout(ui->gridArea);
        gridAreaLayout->setContentsMargins(14, 10, 14, 14);
        gridAreaLayout->setSpacing(0);
        gridAreaLayout->addWidget(ui->gridLayoutWidget, 1);
    }

    if (auto* legacyRoot = cw->findChild<QWidget*>("verticalLayoutWidget", Qt::FindDirectChildrenOnly)) {
        legacyRoot->hide();
        legacyRoot->deleteLater();
    }

    if (ui->horizontalLayout) {
        ui->horizontalLayout->setContentsMargins(12, 0, 12, 0);
        ui->horizontalLayout->setSpacing(12);
        if (ui->labelTitle) ui->horizontalLayout->setAlignment(ui->labelTitle, Qt::AlignVCenter);
        if (ui->comboGrid) ui->horizontalLayout->setAlignment(ui->comboGrid, Qt::AlignVCenter);
    }
    if (ui->gridLayout_tiles) {
        ui->gridLayout_tiles->setContentsMargins(0, 0, 0, 0);
        ui->gridLayout_tiles->setHorizontalSpacing(10);
        ui->gridLayout_tiles->setVerticalSpacing(10);
    }
    if (ui->labelTitle) {
        ui->labelTitle->setText("Camera Grid");
        ui->labelTitle->setMinimumHeight(32);
        ui->labelTitle->setMaximumHeight(32);
        ui->labelTitle->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
        QFont f = ui->labelTitle->font();
        f.setPointSize(15);
        f.setBold(true);
        ui->labelTitle->setFont(f);
    }
    if (ui->comboGrid) {
        ui->comboGrid->setFixedSize(92, 26);
        ui->comboGrid->setSizeAdjustPolicy(QComboBox::AdjustToContents);
        QFont f = ui->comboGrid->font();
        f.setPointSize(10);
        f.setBold(true);
        ui->comboGrid->setFont(f);
    }

    root->invalidate();
    root->activate();
}

// ---------------- ui bind ----------------

void AllCctvWindow::bindUiObjects()
{
    m_tileFrames.resize(16);
    m_titleLabels.resize(16);
    m_videoWidgets.resize(16);

    for (int i = 1; i <= 16; ++i) {
        const QString idx = QString("%1").arg(i, 2, 10, QChar('0'));

        QFrame* frame = findChild<QFrame*>("tile" + idx);
        QLabel* title = findChild<QLabel*>("labelVideoTitle" + idx);
        GstVideoWidget* video = findChild<GstVideoWidget*>("labelVideo_tile" + idx);

        if (!frame || !title || !video) {
            qDebug() << "[BIND MISS]" << i << frame << title << video;
            continue;
        }

        frame->setProperty("tileIndex", i);
        video->setProperty("tileIndex", i);

        // tile ?��? ?�이?�웃 보정 (중복 add 방�?)
        QVBoxLayout* box = ensureFrameVBoxLayout(frame);
        if (!box) continue;

        box->setContentsMargins(1, 1, 1, 1);
        box->setSpacing(0);

        box->removeWidget(title);
        box->removeWidget(video);
        box->addWidget(title);
        box->addWidget(video);
        box->setStretch(0, 0);
        box->setStretch(1, 1);

        title->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        title->setMinimumHeight(26);
        title->setMaximumHeight(26);
        QFont titleFont = title->font();
        titleFont.setPointSize(9);
        titleFont.setBold(true);
        title->setFont(titleFont);
        title->setContentsMargins(12, 0, 12, 0);
        title->setStyleSheet("color:#edf8ff; font-weight:700; background:#1b2435; padding:6px 10px; border-top-left-radius:7px; border-top-right-radius:7px; border-bottom:1px solid #2a3954;");

        video->setStyleSheet("background:black;");
        video->setPreferredOutputWidth(640);
        video->setDisplayMode(i == 3 ? GstVideoWidget::DisplayMode::Grayscale
                                     : (i == 4 ? GstVideoWidget::DisplayMode::EdgeEnhanced
                                               : (i == 5 ? GstVideoWidget::DisplayMode::Inverted
                                                         : (i == 6 ? GstVideoWidget::DisplayMode::ZoomCrop
                                                                   : (i == 7 ? GstVideoWidget::DisplayMode::MotionHighlight
                                                                             : GstVideoWidget::DisplayMode::Normal)))));
        auto* placeholder = new QLabel(video);
        placeholder->setObjectName("lowCostPlaceholder");
        placeholder->setAlignment(Qt::AlignCenter);
        placeholder->setWordWrap(true);
        placeholder->setAttribute(Qt::WA_TransparentForMouseEvents);
        placeholder->setGeometry(20, 20, qMax(120, video->width() - 40), qMax(72, video->height() - 40));
        placeholder->setStyleSheet(
            "QLabel {"
            " color: rgba(225,236,255,0.92);"
            " background: rgba(9,14,23,0.72);"
            " border: 1px dashed rgba(115,146,193,0.5);"
            " border-radius: 10px;"
            " padding: 10px 12px;"
            " font-size: 12px;"
            " font-weight: 600;"
            "}");
        placeholder->hide();
        // video->installEventFilter(this);
        frame->installEventFilter(this);
        title->installEventFilter(this);
        video->installEventFilter(this);   // ???�건 ?�위 eventFilter가 parent�??��??�라가므로�?OK


        frame->setProperty("tileIndex", i);     // ???�레?�에 ?�덱??
        frame->installEventFilter(this);        // ???�레?�이 ?�블?�릭 받게
        frame->setMouseTracking(true);
        frame->setAttribute(Qt::WA_Hover, true);
        frame->setFocusPolicy(Qt::StrongFocus);

        m_tileFrames[i - 1] = frame;
        m_titleLabels[i - 1] = title;
        m_videoWidgets[i - 1] = video;
    }
}

// ---------------- grid ----------------

int AllCctvWindow::maxBoundTiles() const
{
    // ?�몇 개�? 바인???�나???�인??(?�래??방�??�이지�? activeCount??그냥 rows*cols�??�도 ??
    int n = 0;
    for (int i = 0; i < 16; ++i) {
        if (m_tileFrames[i] && m_videoWidgets[i]) ++n;
    }
    return n;
}

void AllCctvWindow::applyGridModeFromCombo()
{
    const int idx = ui->comboGrid->currentIndex();
    if (idx == 0) applyGrid(2, 2);
    else if (idx == 1) applyGrid(3, 3);
    else applyGrid(4, 4);
}

// void AllCctvWindow::applyGrid(int rows, int cols)
// {
//     m_activeRows = rows;
//     m_activeCols = cols;

//     // ???�면???�시?�는 ?�????
//     m_activeCount = rows * cols;

//     rebuildGridLayout(m_activeCount);

//     if (isVisible()) {
//         QTimer::singleShot(0, this, [this]{
//             restartStreamsByVisibility();   // ?�기???�재?��? 4개만??
//         });
//     }
// }
void AllCctvWindow::applyGrid(int rows, int cols)
{
    m_activeRows = rows;
    m_activeCols = cols;

    // ???�면???�시?�는 ?�????
    m_activeCount = rows * cols;

    rebuildGridLayout(m_activeCount);

    if (isVisible()) {
        QTimer::singleShot(0, this, [this]{
            restartStreamsByVisibility();   // ?�기???�재?��? 4개만??
        });
    }
}

// void AllCctvWindow::rebuildGridLayout(int activeCount)
// {
//     QGridLayout* grid = ui->gridLayout_tiles;
//     if (!grid) return;

//     clearGridLayoutOnly(grid);

//     int k = 0;
//     for (int r = 0; r < m_activeRows; ++r) {
//         for (int c = 0; c < m_activeCols; ++c) {
//             if (k >= activeCount) break;
//             if (k < 16 && m_tileFrames[k]) {
//                 m_tileFrames[k]->show();
//                 grid->addWidget(m_tileFrames[k], r, c);
//             }
//             ++k;
//         }
//     }

//     for (int i = activeCount; i < 16; ++i) {
//         if (m_tileFrames[i]) m_tileFrames[i]->hide();
//     }

//     for (int r = 0; r < 4; ++r) grid->setRowStretch(r, 0);
//     for (int c = 0; c < 4; ++c) grid->setColumnStretch(c, 0);
//     for (int r = 0; r < m_activeRows; ++r) grid->setRowStretch(r, 1);
//     for (int c = 0; c < m_activeCols; ++c) grid->setColumnStretch(c, 1);

//     grid->invalidate();
//     grid->activate();
// }


void AllCctvWindow::rebuildGridLayout(int activeCount)
{
    QGridLayout* grid = ui->gridLayout_tiles;
    if (!grid) return;

    clearGridLayoutOnly(grid);

    int k = 0;
    for (int r = 0; r < m_activeRows; ++r) {
        for (int c = 0; c < m_activeCols; ++c) {
            if (k >= activeCount) break;

            if (k < 16 && m_tileFrames[k]) {
                m_tileFrames[k]->show();
                grid->addWidget(m_tileFrames[k], r, c);
            }
            ++k;
        }
    }

    // activeCount 밖�? hide
    for (int i = activeCount; i < 16; ++i) {
        if (m_tileFrames[i]) m_tileFrames[i]->hide();
    }

    // stretch
    for (int r = 0; r < 4; ++r) grid->setRowStretch(r, 0);
    for (int c = 0; c < 4; ++c) grid->setColumnStretch(c, 0);
    for (int r = 0; r < m_activeRows; ++r) grid->setRowStretch(r, 1);
    for (int c = 0; c < m_activeCols; ++c) grid->setColumnStretch(c, 1);

    grid->invalidate();
    grid->activate();
}

// ---------------- streams ----------------

void AllCctvWindow::setStreams(const QMap<int, QString>& rtspByTile,
                               const QMap<int, QString>& titleByTile)
{
    m_rtsp = rtspByTile;
    m_titles = titleByTile;

    for (int i = 1; i <= 16; ++i) {
        if (m_titleLabels[i - 1])
            m_titleLabels[i - 1]->setText(m_titles.value(i, QString("CCTV %1").arg(i)));
        if (m_videoWidgets[i - 1])
            m_videoWidgets[i - 1]->setDisplayMode(
                i == 3 ? GstVideoWidget::DisplayMode::Grayscale
                       : (i == 4 ? GstVideoWidget::DisplayMode::EdgeEnhanced
                                 : (i == 5 ? GstVideoWidget::DisplayMode::Inverted
                                           : (i == 6 ? GstVideoWidget::DisplayMode::ZoomCrop
                                                     : (i == 7 ? GstVideoWidget::DisplayMode::MotionHighlight
                                                               : GstVideoWidget::DisplayMode::Normal)))));
    }

    // show ?�에??stop�??�고, showEvent?�서 start
    closeAllStreams();

    if (isVisible()) {
        restartStreamsByVisibility();
    }
}

void AllCctvWindow::restartStreamsByVisibility()
{
    // ???�제�??�생???�???�는 최�? 4�?
    const int playCount = qMin(4, m_activeCount);

    // 1) playCount �?= ?�생?�면 ???�는 ?�???� stop
    for (int i = playCount + 1; i <= 16; ++i) {
        auto* w = m_videoWidgets[i - 1];
        if (!w) continue;
        auto* placeholder = w->findChild<QLabel*>("lowCostPlaceholder", Qt::FindDirectChildrenOnly);

        if (!m_playingUrl[i - 1].isEmpty()) {
            w->stopStream();
            m_playingUrl[i - 1].clear();
        }
        if (placeholder) {
            placeholder->setGeometry(20, 20, qMax(120, w->width() - 40), qMax(72, w->height() - 40));
            if (i >= 5 && i <= 7) {
                placeholder->setText(QString("%1\nPreview paused for low CPU mode")
                                         .arg(m_titles.value(i, QString("CCTV %1").arg(i))));
                placeholder->show();
                placeholder->raise();
            } else {
                placeholder->hide();
            }
        }
    }

    // 2) 1..playCount�??�생 관�?
    for (int i = 1; i <= playCount; ++i) {
        auto* w = m_videoWidgets[i - 1];
        if (!w) continue;
        auto* placeholder = w->findChild<QLabel*>("lowCostPlaceholder", Qt::FindDirectChildrenOnly);
        if (placeholder) placeholder->hide();

        const QString url = m_rtsp.value(i).trimmed();

        // url ?�으�?stop (=> 검???�???��?)
        if (url.isEmpty()) {
            if (!m_playingUrl[i - 1].isEmpty()) {
                w->stopStream();
                m_playingUrl[i - 1].clear();
            }
            if (placeholder) {
                placeholder->setGeometry(20, 20, qMax(120, w->width() - 40), qMax(72, w->height() - 40));
                placeholder->setText(QString("%1\nNo stream assigned")
                                         .arg(m_titles.value(i, QString("CCTV %1").arg(i))));
                placeholder->show();
                placeholder->raise();
            }
            continue;
        }

        // url ?�일?�면 ?�시??금�?
        if (m_playingUrl[i - 1] == url) continue;

        // url 바뀌면 ?�시??
        w->stopStream();
        w->startStream(url);
        m_playingUrl[i - 1] = url;
    }

    // 3) playCount+1..m_activeCount(= ?�면??보이지�??�생 ?????� ?�그??검?��?
    //    ?�기??굳이 stop???????�요 ?�음(?�에??16까�? stop?�음)
}



void AllCctvWindow::closeAllStreams()
{
    // detach 경고 방�??? std::as_const
    for (auto *w : std::as_const(m_videoWidgets)) {
        if (w) {
            w->stopStream();
            if (auto* placeholder = w->findChild<QLabel*>("lowCostPlaceholder", Qt::FindDirectChildrenOnly)) {
                placeholder->hide();
            }
        }
    }
}

// ---------------- events ----------------
bool AllCctvWindow::eventFilter(QObject* watched, QEvent* event)
{
    if (event->type() == QEvent::Resize) {
        auto* video = qobject_cast<GstVideoWidget*>(watched);
        if (video) {
            if (auto* placeholder = video->findChild<QLabel*>("lowCostPlaceholder", Qt::FindDirectChildrenOnly)) {
                placeholder->setGeometry(20, 20, qMax(120, video->width() - 40), qMax(72, video->height() - 40));
            }
        }
    }

    if (event->type() == QEvent::MouseButtonDblClick) {

        // ??watched가 child�??�어?�??tile ?�레?�까지 ?�라가??tileIndex 찾기
        QWidget* w = qobject_cast<QWidget*>(watched);
        if (!w) return QMainWindow::eventFilter(watched, event);

        QWidget* cur = w;
        bool ok = false;
        int tile = -1;

        while (cur) {
            QVariant v = cur->property("tileIndex");
            if (v.isValid()) {
                tile = v.toInt(&ok);
                if (ok) break;
            }
            cur = cur->parentWidget();
        }

        if (!ok || tile < 1 || tile > m_activeCount) return true;

        const QString url = m_rtsp.value(tile).trimmed();
        if (url.isEmpty()) return true;

        const QString title = m_titles.value(tile, QString("CCTV %1").arg(tile));

        openFullScreenViewer(title, url);
        return true;
    }

    return QMainWindow::eventFilter(watched, event);
}



// bool AllCctvWindow::eventFilter(QObject* watched, QEvent* event)
// {
//     if (event->type() == QEvent::MouseButtonDblClick) {
//         auto* w = qobject_cast<GstVideoWidget*>(watched);
//         if (!w) return QMainWindow::eventFilter(watched, event);

//         bool ok = false;
//         int tile = w->property("tileIndex").toInt(&ok);
//         if (!ok || tile < 1 || tile > m_activeCount) return true;

//         const QString url = m_rtsp.value(tile);
//         if (url.isEmpty()) return true;

//         const QString title = m_titles.value(tile, QString("CCTV %1").arg(tile));
//         if (m_zoom) m_zoom->openStream(title, url);
//         return true;
//     }
//     return QMainWindow::eventFilter(watched, event);
// }


void AllCctvWindow::resizeEvent(QResizeEvent* e)
{
    QMainWindow::resizeEvent(e);
}

void AllCctvWindow::showEvent(QShowEvent* e)
{
    QMainWindow::showEvent(e);

    // ??show ?�후 0ms ?�에 ?�작 (winId ?�정??
    QTimer::singleShot(0, this, [this]{
        //dumpTopLeftSmallWidgets(this); ui?�버�?
        restartStreamsByVisibility();
    });
}

void AllCctvWindow::onGridChanged(int)
{
    applyGridModeFromCombo();
}

void AllCctvWindow::onFullScreen()
{
    if (isMaximized()) showNormal();
    else showMaximized();

    QTimer::singleShot(0, this, [this]{
        restartStreamsByVisibility();
    });
}

void AllCctvWindow::onCloseClicked()
{
    close();
}

void AllCctvWindow::openFullScreenViewer(const QString& title, const QString& url)
{
    if (!m_full) {
        m_full = new FullscreenViewer(nullptr);
        m_full->setAttribute(Qt::WA_DeleteOnClose, false);
    }
    m_full->openStreamFull(title, url);
}


//-------------debug------------------
// void AllCctvWindow::dumpTopLeftSmallWidgets(QWidget* root)
// {
//     const QRect area(0, 0, 120, 120); // 좌상??근처�?
//     auto ws = root->findChildren<QWidget*>();
//     for (auto* w : ws) {
//         if (!w || !w->isVisible()) continue;

//         QRect g = w->geometry();
//         // "?�고" + "좌상??근처"???�는 ?�만 추림
//         if (g.width() <= 120 && g.height() <= 120 && area.intersects(g)) {
//             qDebug() << "[SUSPECT]"
//                      << w->metaObject()->className()
//                      << "name=" << w->objectName()
//                      << "geom=" << g
//                      << "parent=" << (w->parentWidget() ? w->parentWidget()->objectName() : "null");
//         }
//     }
// }











void AllCctvWindow::setupCustomTitleBar()
{
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint);

    m_titleBar = new TitleBarWidget(this);
    m_titleBar->setTitleText(windowTitle());
    m_titleBar->setMaximized(isMaximized());

    setMenuWidget(m_titleBar);

    connect(this, &QWidget::windowTitleChanged,
            m_titleBar, &TitleBarWidget::setTitleText);
    connect(m_titleBar, &TitleBarWidget::minimizeRequested,
            this, &QWidget::showMinimized);
    connect(m_titleBar, &TitleBarWidget::maximizeRequested,
            this, &QWidget::showMaximized);
    connect(m_titleBar, &TitleBarWidget::restoreRequested,
            this, &QWidget::showNormal);
    connect(m_titleBar, &TitleBarWidget::closeRequested,
            this, &QWidget::close);
}
bool AllCctvWindow::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
    if (FramelessHelper::handleNativeEvent(this, m_titleBar, eventType, message, result))
        return true;
    return QMainWindow::nativeEvent(eventType, message, result);
}

void AllCctvWindow::changeEvent(QEvent* event)
{
    if (event && event->type() == QEvent::WindowStateChange) {
        if (m_titleBar) m_titleBar->setMaximized(isMaximized());
    }
    QMainWindow::changeEvent(event);
}









