#include "AllCctvWindow.h"
#include "ui_AllCctvWindow.h"
#include "CctvZoomWindow.h"
#include "GstVideoWidget.h"   // ???Œì¼ëª??€?Œë¬¸???¤ì œ?€ ?™ì¼?˜ê²Œ!
#include "FullscreenViewer.h"
#include <QEvent>
#include <QMouseEvent>
#include <QComboBox>
#include <QVBoxLayout>
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
        delete item; // widget?€ ?? œ ????
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

    setStyleSheet(R"(
QMainWindow, QWidget#centralWidget {
    background-color: #6d6a64;
    color: #333333;
}
QWidget#topBar, QWidget#gridArea {
    background-color: #c2b8b0;
}
QGroupBox::title {
    color: #00E5FF;
    font-weight: 700;
}
QLabel#labelTitle {
    color: #333333;
    font-weight: 700;
    font-size: 15px;
}
QComboBox, QPushButton {
    background-color: #e3ddda;
    color: #333333;
    border: 1px solid #665950;
    border-radius: 2px;
    min-height: 22px;
    padding: 1px 6px;
}
QPushButton:hover, QComboBox:hover {
    background-color: #bcb2aa;
}
QPushButton:pressed {
    background-color: #b3a79e;
}
QFrame {
    background-color: #534842;
    border: 1px solid #988575;
    border-radius: 3px;
}
QLabel {
    background-color: #c2b8b0;
    color: #333333;
    font-size: 13px;
    border: none;
}
)");

    ensureRootLayout();

    connect(ui->comboGrid,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &AllCctvWindow::onGridChanged);

    bindUiObjects();

    // ??ë°˜ë“œ??16?¼ë¡œ ê³ ì •
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

    root->addWidget(ui->topBar);
    root->addWidget(ui->gridArea);

    root->setStretch(0, 0);
    root->setStretch(1, 1);

    ui->topBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ui->topBar->setMinimumHeight(42);

    ui->gridArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

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

        // tile ?´ë? ?ˆì´?„ì›ƒ ë³´ì • (ì¤‘ë³µ add ë°©ì?)
        QVBoxLayout* box = ensureFrameVBoxLayout(frame);
        if (!box) continue;

        box->setContentsMargins(0, 0, 0, 0);
        box->setSpacing(0);

        box->removeWidget(title);
        box->removeWidget(video);
        box->addWidget(title);
        box->addWidget(video);
        box->setStretch(0, 0);
        box->setStretch(1, 1);

        title->setAlignment(Qt::AlignCenter);        title->setStyleSheet("color:#f8f2ec; font-weight:700; background:#534842; padding:3px 4px;");

        video->setStyleSheet("background:black;");
        // video->installEventFilter(this);
        frame->installEventFilter(this);
        title->installEventFilter(this);
        video->installEventFilter(this);   // ???´ê±´ ?œìœ„ eventFilterê°€ parentë¡??€ê³??¬ë¼ê°€ë¯€ë¡œâ€?OK


        frame->setProperty("tileIndex", i);     // ???„ë ˆ?„ì— ?¸ë±??
        frame->installEventFilter(this);        // ???„ë ˆ?„ì´ ?”ë¸”?´ë¦­ ë°›ê²Œ
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
    // ?œëª‡ ê°œê? ë°”ì¸???ë‚˜???•ì¸??(?¬ë˜??ë°©ì??©ì´ì§€ë§? activeCount??ê·¸ëƒ¥ rows*colsë¡??¬ë„ ??
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

//     // ???”ë©´???œì‹œ?˜ëŠ” ?€????
//     m_activeCount = rows * cols;

//     rebuildGridLayout(m_activeCount);

//     if (isVisible()) {
//         QTimer::singleShot(0, this, [this]{
//             restartStreamsByVisibility();   // ?¬ê¸°???œì¬?ì? 4ê°œë§Œ??
//         });
//     }
// }
void AllCctvWindow::applyGrid(int rows, int cols)
{
    m_activeRows = rows;
    m_activeCols = cols;

    // ???”ë©´???œì‹œ?˜ëŠ” ?€????
    m_activeCount = rows * cols;

    rebuildGridLayout(m_activeCount);

    if (isVisible()) {
        QTimer::singleShot(0, this, [this]{
            restartStreamsByVisibility();   // ?¬ê¸°???œì¬?ì? 4ê°œë§Œ??
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

    // activeCount ë°–ì? hide
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
    }

    // show ?„ì—??stopë§??˜ê³ , showEvent?ì„œ start
    closeAllStreams();

    if (isVisible()) {
        restartStreamsByVisibility();
    }
}

void AllCctvWindow::restartStreamsByVisibility()
{
    // ???¤ì œë¡??¬ìƒ???€???˜ëŠ” ìµœë? 4ê°?
    const int playCount = qMin(4, m_activeCount);

    // 1) playCount ë°?= ?¬ìƒ?˜ë©´ ???˜ëŠ” ?€???€ stop
    for (int i = playCount + 1; i <= 16; ++i) {
        auto* w = m_videoWidgets[i - 1];
        if (!w) continue;

        if (!m_playingUrl[i - 1].isEmpty()) {
            w->stopStream();
            m_playingUrl[i - 1].clear();
        }
    }

    // 2) 1..playCountë§??¬ìƒ ê´€ë¦?
    for (int i = 1; i <= playCount; ++i) {
        auto* w = m_videoWidgets[i - 1];
        if (!w) continue;

        const QString url = m_rtsp.value(i).trimmed();

        // url ?†ìœ¼ë©?stop (=> ê²€???€??? ì?)
        if (url.isEmpty()) {
            if (!m_playingUrl[i - 1].isEmpty()) {
                w->stopStream();
                m_playingUrl[i - 1].clear();
            }
            continue;
        }

        // url ?™ì¼?˜ë©´ ?¬ì‹œ??ê¸ˆì?
        if (m_playingUrl[i - 1] == url) continue;

        // url ë°”ë€Œë©´ ?¬ì‹œ??
        w->stopStream();
        w->startStream(url);
        m_playingUrl[i - 1] = url;
    }

    // 3) playCount+1..m_activeCount(= ?”ë©´??ë³´ì´ì§€ë§??¬ìƒ ?????€ ?œê·¸??ê²€?•â€?
    //    ?¬ê¸°??êµ³ì´ stop???????„ìš” ?†ìŒ(?„ì—??16ê¹Œì? stop?ˆìŒ)
}



void AllCctvWindow::closeAllStreams()
{
    // detach ê²½ê³  ë°©ì??? std::as_const
    for (auto *w : std::as_const(m_videoWidgets)) {
        if (w) w->stopStream();
    }
}

// ---------------- events ----------------
bool AllCctvWindow::eventFilter(QObject* watched, QEvent* event)
{
    if (event->type() == QEvent::MouseButtonDblClick) {

        // ??watchedê°€ childë¡??¤ì–´?€??tile ?„ë ˆ?„ê¹Œì§€ ?¬ë¼ê°€??tileIndex ì°¾ê¸°
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

    // ??show ?´í›„ 0ms ?¤ì— ?œì‘ (winId ?ˆì •??
    QTimer::singleShot(0, this, [this]{
        //dumpTopLeftSmallWidgets(this); ui?”ë²„ê¹?
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
//     const QRect area(0, 0, 120, 120); // ì¢Œìƒ??ê·¼ì²˜ë§?
//     auto ws = root->findChildren<QWidget*>();
//     for (auto* w : ws) {
//         if (!w || !w->isVisible()) continue;

//         QRect g = w->geometry();
//         // "?‘ê³ " + "ì¢Œìƒ??ê·¼ì²˜"???ˆëŠ” ?ˆë§Œ ì¶”ë¦¼
//         if (g.width() <= 120 && g.height() <= 120 && area.intersects(g)) {
//             qDebug() << "[SUSPECT]"
//                      << w->metaObject()->className()
//                      << "name=" << w->objectName()
//                      << "geom=" << g
//                      << "parent=" << (w->parentWidget() ? w->parentWidget()->objectName() : "null");
//         }
//     }
// }










