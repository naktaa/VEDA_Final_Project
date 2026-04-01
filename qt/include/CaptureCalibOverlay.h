#ifndef CAPTURECALIBOVERLAY_H
#define CAPTURECALIBOVERLAY_H

#include <QImage>
#include <QPointF>
#include <QTransform>
#include <QVector>
#include <QWidget>

class QPushButton;

class CaptureCalibOverlay : public QWidget
{
    Q_OBJECT
public:
    explicit CaptureCalibOverlay(QWidget* parent = nullptr);

    void setImage(const QImage& img);
    void setInitialPoints(const QVector<QPointF>& pts);
    void setWorldPolygon(const QVector<QPointF>& worldPts);
    void setHomography(const QTransform& worldToImage, bool valid);
    void setMapOverlayVisible(bool visible);
    bool mapOverlayVisible() const { return m_mapVisible; }

signals:
    void calibrationSaved(const QVector<QPointF>& imagePoints);
    void closed();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;

private:
    QPointF widgetToImage(const QPointF& p) const;
    QPointF imageToWidget(const QPointF& p) const;
    int nearestPointIndex(const QPointF& imgPt, double thPx) const;
    bool hasAllPoints() const;
    void updateButtons();
    void layoutButtons();

private:
    QImage m_image;
    QRectF m_drawRect;
    QVector<QPointF> m_points;
    int m_nextIndex = 0;

    QPushButton* m_btnReset = nullptr;
    QPushButton* m_btnSave = nullptr;
    QPushButton* m_btnMap = nullptr;
    QPushButton* m_btnClose = nullptr;

    QVector<QPointF> m_worldPts;
    QTransform m_worldToImage;
    bool m_hasHomography = false;
    bool m_mapVisible = true;
    int m_dragIndex = -1;
    bool m_dragging = false;
};

#endif
