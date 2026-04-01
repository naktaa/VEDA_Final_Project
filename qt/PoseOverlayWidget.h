#ifndef POSEOVERLAYWIDGET_H
#define POSEOVERLAYWIDGET_H

#include <QRect>
#include <QSize>
#include <QWidget>

#include "PoseTypes.h"

class QPainter;

class PoseOverlayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PoseOverlayWidget(QWidget *parent = nullptr);

    void setPoseFrame(const PoseFrame &frame);
    void setVideoGeometry(const QRect &displayRect, const QSize &sourceFrameSize = QSize());

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QPointF mapPoint(const QPointF &src) const;
    QRectF mapRect(const QRect &src) const;
    void drawSkeleton(QPainter &p);
    void drawBBox(QPainter &p);
    void drawLabel(QPainter &p);

private:
    PoseFrame m_frame;
    QRect m_videoRect;
    QSize m_sourceFrameSize;
};

#endif
