#include "CaptureCalibOverlay.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>

CaptureCalibOverlay::CaptureCalibOverlay(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet("background-color: rgba(0,0,0,140);");

    m_points.resize(4, QPointF(-1, -1));

    m_btnReset = new QPushButton("Reset", this);
    m_btnSave = new QPushButton("Save", this);
    m_btnMap = new QPushButton("Map ON", this);
    m_btnClose = new QPushButton("Close", this);

    const char* primaryBtnStyle = R"(
QPushButton {
    background-color: rgba(255, 255, 255, 185);
    color: #1f1f1f;
    border: 1px solid rgba(255, 255, 255, 220);
    border-radius: 4px;
    padding: 4px 8px;
    font-weight: 600;
}
QPushButton:hover {
    background-color: rgba(255, 255, 255, 220);
}
QPushButton:pressed {
    background-color: rgba(235, 235, 235, 230);
}
QPushButton:disabled {
    background-color: rgba(255, 255, 255, 120);
    color: rgba(31, 31, 31, 130);
    border: 1px solid rgba(255, 255, 255, 140);
}
)";
    const char* secondaryBtnStyle = R"(
QPushButton {
    background-color: rgba(255, 255, 255, 120);
    color: #1f1f1f;
    border: 1px solid rgba(255, 255, 255, 170);
    border-radius: 4px;
    padding: 4px 8px;
    font-weight: 600;
}
QPushButton:hover {
    background-color: rgba(255, 255, 255, 150);
}
QPushButton:pressed {
    background-color: rgba(235, 235, 235, 170);
}
QPushButton:disabled {
    background-color: rgba(255, 255, 255, 90);
    color: rgba(31, 31, 31, 110);
    border: 1px solid rgba(255, 255, 255, 110);
}
)";
    m_btnMap->setStyleSheet(primaryBtnStyle);
    m_btnSave->setStyleSheet(primaryBtnStyle);
    m_btnReset->setStyleSheet(secondaryBtnStyle);
    m_btnClose->setStyleSheet(secondaryBtnStyle);

    connect(m_btnReset, &QPushButton::clicked, this, [this]{
        m_points.fill(QPointF(-1, -1));
        m_nextIndex = 0;
        updateButtons();
        update();
    });
    connect(m_btnSave, &QPushButton::clicked, this, [this]{
        if (!hasAllPoints()) return;
        emit calibrationSaved(m_points);
    });
    connect(m_btnMap, &QPushButton::clicked, this, [this]{
        m_mapVisible = !m_mapVisible;
        if (m_btnMap) m_btnMap->setText(m_mapVisible ? "Map ON" : "Map OFF");
        update();
    });
    connect(m_btnClose, &QPushButton::clicked, this, [this]{
        hide();
        emit closed();
    });

    updateButtons();
}

void CaptureCalibOverlay::setImage(const QImage& img)
{
    m_image = img;
    update();
}

void CaptureCalibOverlay::setInitialPoints(const QVector<QPointF>& pts)
{
    if (pts.size() == 4) {
        m_points = pts;
        m_nextIndex = 4;
    } else {
        m_points.fill(QPointF(-1, -1));
        m_nextIndex = 0;
    }
    updateButtons();
    update();
}

void CaptureCalibOverlay::setWorldPolygon(const QVector<QPointF>& worldPts)
{
    m_worldPts = worldPts;
    update();
}

void CaptureCalibOverlay::setHomography(const QTransform& worldToImage, bool valid)
{
    m_worldToImage = worldToImage;
    m_hasHomography = valid;
    update();
}

void CaptureCalibOverlay::setMapOverlayVisible(bool visible)
{
    m_mapVisible = visible;
    if (m_btnMap) m_btnMap->setText(m_mapVisible ? "Map ON" : "Map OFF");
    update();
}

void CaptureCalibOverlay::paintEvent(QPaintEvent* e)
{
    QWidget::paintEvent(e);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    if (m_image.isNull()) return;

    QSize target = m_image.size();
    target.scale(size(), Qt::KeepAspectRatio);
    m_drawRect = QRectF(QPointF(0, 0), target);
    m_drawRect.moveCenter(rect().center());

    p.drawImage(m_drawRect, m_image);

    // In capture calibration mode, draw editable guide lines from picked image points.
    // This avoids confusion with a second projected line that can look fixed.
    if (m_mapVisible) {
        p.setPen(QPen(QColor(0, 220, 255, 230), 3));
        if (m_points[0].x() >= 0 && m_points[1].x() >= 0)
            p.drawLine(imageToWidget(m_points[0]), imageToWidget(m_points[1]));
        if (m_points[1].x() >= 0 && m_points[2].x() >= 0)
            p.drawLine(imageToWidget(m_points[1]), imageToWidget(m_points[2]));
        if (m_points[2].x() >= 0 && m_points[3].x() >= 0)
            p.drawLine(imageToWidget(m_points[2]), imageToWidget(m_points[3]));
        if (m_points[3].x() >= 0 && m_points[0].x() >= 0)
            p.drawLine(imageToWidget(m_points[3]), imageToWidget(m_points[0]));
    }

    static const char* labels[4] = {"10", "11", "12", "13"};
    p.setPen(QPen(QColor(255, 255, 255, 240), 2));
    p.setBrush(QColor(255, 80, 80, 220));
    for (int i = 0; i < m_points.size(); ++i) {
        if (m_points[i].x() < 0 || m_points[i].y() < 0) continue;
        const QPointF wp = imageToWidget(m_points[i]);
        p.drawEllipse(wp, 9, 9);
        p.drawText(wp + QPointF(10, -10), QString(labels[i]));
    }

    p.setPen(Qt::white);
    p.drawText(16, 24, "Set points 10 -> 11 -> 12 -> 13, drag to refine, Map ON/OFF toggles guide lines");
}

void CaptureCalibOverlay::mousePressEvent(QMouseEvent* e)
{
    if (m_image.isNull() || !e) return;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QPointF pos = e->position();
#else
    const QPointF pos = e->localPos();
#endif
    if (!m_drawRect.contains(pos)) return;

    const QPointF ip = widgetToImage(pos);
    if (ip.x() < 0 || ip.y() < 0) return;

    if (m_nextIndex < 4) {
        m_points[m_nextIndex] = ip;
        ++m_nextIndex;
    } else {
        const int idx = nearestPointIndex(ip, 36.0);
        if (idx >= 0) {
            m_dragIndex = idx;
            m_dragging = true;
            m_points[idx] = ip;
        }
    }

    updateButtons();
    update();
    e->accept();
}

void CaptureCalibOverlay::mouseMoveEvent(QMouseEvent* e)
{
    if (!e) return;
    if (!m_dragging || m_dragIndex < 0 || m_dragIndex >= m_points.size()) {
        QWidget::mouseMoveEvent(e);
        return;
    }
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QPointF pos = e->position();
#else
    const QPointF pos = e->localPos();
#endif
    if (!m_drawRect.contains(pos)) return;

    const QPointF ip = widgetToImage(pos);
    if (ip.x() < 0 || ip.y() < 0) return;
    m_points[m_dragIndex] = ip;
    update();
    e->accept();
}

void CaptureCalibOverlay::mouseReleaseEvent(QMouseEvent* e)
{
    if (m_dragging) {
        m_dragging = false;
        m_dragIndex = -1;
        if (e) e->accept();
        return;
    }
    QWidget::mouseReleaseEvent(e);
}

void CaptureCalibOverlay::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    layoutButtons();
}

QPointF CaptureCalibOverlay::widgetToImage(const QPointF& p) const
{
    if (m_image.isNull() || m_drawRect.width() <= 0 || m_drawRect.height() <= 0)
        return QPointF(-1, -1);

    const double nx = (p.x() - m_drawRect.left()) / m_drawRect.width();
    const double ny = (p.y() - m_drawRect.top()) / m_drawRect.height();
    return QPointF(nx * m_image.width(), ny * m_image.height());
}

QPointF CaptureCalibOverlay::imageToWidget(const QPointF& p) const
{
    if (m_image.isNull() || m_drawRect.width() <= 0 || m_drawRect.height() <= 0)
        return QPointF();

    const double nx = p.x() / m_image.width();
    const double ny = p.y() / m_image.height();
    return QPointF(m_drawRect.left() + nx * m_drawRect.width(),
                   m_drawRect.top() + ny * m_drawRect.height());
}

int CaptureCalibOverlay::nearestPointIndex(const QPointF& imgPt, double thPx) const
{
    int best = -1;
    double bestD = thPx;
    for (int i = 0; i < m_points.size(); ++i) {
        if (m_points[i].x() < 0 || m_points[i].y() < 0) continue;
        const double d = QLineF(imgPt, m_points[i]).length();
        if (d <= bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

bool CaptureCalibOverlay::hasAllPoints() const
{
    for (const auto& p : m_points) {
        if (p.x() < 0 || p.y() < 0) return false;
    }
    return true;
}

void CaptureCalibOverlay::updateButtons()
{
    if (m_btnSave) m_btnSave->setEnabled(hasAllPoints());
}

void CaptureCalibOverlay::layoutButtons()
{
    const int w = 90;
    const int h = 34;
    const int gap = 8;
    const int y = height() - h - 14;
    int x = width() - (w * 4 + gap * 3) - 14;

    if (m_btnMap) m_btnMap->setGeometry(x, y, w, h);
    x += w + gap;
    if (m_btnSave) m_btnSave->setGeometry(x, y, w, h);
    x += w + gap;
    if (m_btnReset) m_btnReset->setGeometry(x, y, w, h);
    x += w + gap;
    if (m_btnClose) m_btnClose->setGeometry(x, y, w, h);
}
