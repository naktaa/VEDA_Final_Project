#ifndef HUMANBOXOVERLAYWIDGET_H
#define HUMANBOXOVERLAYWIDGET_H

#include <QRect>
#include <QSize>
#include <QVector>
#include <QWidget>

class HumanBoxOverlayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HumanBoxOverlayWidget(QWidget* parent = nullptr);

    void setHumanBox(const QRect& bbox, bool visible);
    void setHumanBoxes(const QVector<QRect>& boxes, bool visible);
    void setVideoGeometry(const QRect& displayRect, const QSize& sourceFrameSize = QSize());
    void setBoxSourceSize(const QSize& sourceFrameSize);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QRectF mapRect(const QRect& src) const;

private:
    QVector<QRect> m_boxes;
    bool m_visible = false;
    QRect m_videoRect;
    QSize m_sourceFrameSize;
};

#endif
