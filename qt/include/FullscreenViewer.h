#ifndef FULLSCREENVIEWER_H
#define FULLSCREENVIEWER_H
#pragma once

#include <QDialog>
#include <QString>

class QLabel;
class GstVideoWidget;
class CctvZoomWindow;

class FullscreenViewer : public QDialog
{
    Q_OBJECT
public:
    explicit FullscreenViewer(QWidget* parent = nullptr);
    ~FullscreenViewer();

    void openStreamFull(const QString& title, const QString& url);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void closeEvent(QCloseEvent* e) override;

private:
    void start(const QString& url);
    void stop();

private:
    QLabel* m_title = nullptr;
    GstVideoWidget* m_video = nullptr;

    QString m_playingUrl;
    QString m_titleText;
    QString m_url;

    // ✅ 2차 확대용(별도 창)
    CctvZoomWindow* m_zoom = nullptr;
};

#endif // FULLSCREENVIEWER_H
