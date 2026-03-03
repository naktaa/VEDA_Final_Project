#ifndef ALLCCTVWINDOW_H
#define ALLCCTVWINDOW_H

#include <QMainWindow>
#include <QFrame>
#include <QLabel>
#include <QGridLayout>
#include <QMap>
#include <QVector>

QT_BEGIN_NAMESPACE
namespace Ui { class AllCctvWindow; }
QT_END_NAMESPACE

class CctvZoomWindow;
class GstVideoWidget;
class FullscreenViewer;
class AllCctvWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit AllCctvWindow(QWidget* parent = nullptr);
    ~AllCctvWindow();

    // key: 1..16
    void setStreams(const QMap<int, QString>& rtspByTile,
                    const QMap<int, QString>& titleByTile);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void resizeEvent(QResizeEvent* e) override;
    void showEvent(QShowEvent* e) override;

private slots:
    void onGridChanged(int index);
    void onFullScreen();
    void onCloseClicked();

private:
    /* ---------- layout / ui ---------- */
    void ensureRootLayout();
    void bindUiObjects();

    void applyGridModeFromCombo();
    void applyGrid(int rows, int cols);
    void rebuildGridLayout(int activeCount);

    int  maxBoundTiles() const;

    /* ---------- stream control ---------- */
    void openActiveStreams();
    void closeAllStreams();
    void stopStreamsOutsideActive();
    void restartStreamsByVisibility();

    QVector<int> m_startGen;   // tile별 start 요청 세대번호 (race 방지)
    bool m_restartPending = false;

    void scheduleRestartStreams();
    void openFullScreenViewer(const QString& title, const QString& url);
    //void dumpTopLeftSmallWidgets(QWidget* root);

private:
    Ui::AllCctvWindow* ui = nullptr;
    FullscreenViewer* m_full = nullptr;

    /* ---------- tiles ---------- */
    QVector<QFrame*>         m_tileFrames;    // 16
    QVector<QLabel*>         m_titleLabels;   // 16 (labelVideoTitleXX)
    QVector<GstVideoWidget*> m_videoWidgets;  // 16 (labelVideo_tileXX → Promote)
    QVector<QString> m_playingUrl; // size 16

    /* ---------- zoom ---------- */
    CctvZoomWindow* m_zoom = nullptr;

    /* ---------- stream info ---------- */
    QMap<int, QString> m_rtsp;    // 1..16
    QMap<int, QString> m_titles;  // 1..16

    /* ---------- grid state ---------- */
    int m_activeRows  = 4;
    int m_activeCols  = 4;
    int m_activeCount = 16;
};

#endif // ALLCCTVWINDOW_H
