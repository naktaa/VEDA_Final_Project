#include "FramelessHelper.h"

#include <QAbstractButton>
#include <QWidget>

#ifdef Q_OS_WIN
#include <windows.h>
#include <windowsx.h>
#ifndef GET_X_LPARAM
#define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#endif
#ifndef GET_Y_LPARAM
#define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))
#endif
#endif

bool FramelessHelper::handleNativeEvent(QWidget* window, QWidget* titleBar,
                                        const QByteArray& eventType,
                                        void* message, qintptr* result)
{
#ifdef Q_OS_WIN
    if (!window || !result) return false;
    if (eventType != "windows_generic_MSG" && eventType != "windows_dispatcher_MSG")
        return false;

    MSG* msg = static_cast<MSG*>(message);
    if (!msg) return false;

    if (msg->message == WM_GETMINMAXINFO) {
        MINMAXINFO* mmi = reinterpret_cast<MINMAXINFO*>(msg->lParam);
        HMONITOR monitor = MonitorFromWindow(reinterpret_cast<HWND>(window->winId()),
                                            MONITOR_DEFAULTTONEAREST);
        MONITORINFO mi;
        mi.cbSize = sizeof(mi);
        if (GetMonitorInfo(monitor, &mi)) {
            const RECT work = mi.rcWork;
            const RECT monitorRect = mi.rcMonitor;
            mmi->ptMaxPosition.x = work.left - monitorRect.left;
            mmi->ptMaxPosition.y = work.top - monitorRect.top;
            mmi->ptMaxSize.x = work.right - work.left;
            mmi->ptMaxSize.y = work.bottom - work.top;
            *result = 0;
            return true;
        }
    }

    if (msg->message == WM_NCHITTEST) {
        const LONG borderWidth = 8;
        RECT winRect;
        GetWindowRect(reinterpret_cast<HWND>(window->winId()), &winRect);

        const long x = GET_X_LPARAM(msg->lParam) - winRect.left;
        const long y = GET_Y_LPARAM(msg->lParam) - winRect.top;
        const int width = winRect.right - winRect.left;
        const int height = winRect.bottom - winRect.top;

        const bool onLeft = x >= 0 && x < borderWidth;
        const bool onRight = x < width && x >= width - borderWidth;
        const bool onTop = y >= 0 && y < borderWidth;
        const bool onBottom = y < height && y >= height - borderWidth;

        if (onTop && onLeft) {
            *result = HTTOPLEFT;
            return true;
        }
        if (onTop && onRight) {
            *result = HTTOPRIGHT;
            return true;
        }
        if (onBottom && onLeft) {
            *result = HTBOTTOMLEFT;
            return true;
        }
        if (onBottom && onRight) {
            *result = HTBOTTOMRIGHT;
            return true;
        }
        if (onTop) {
            *result = HTTOP;
            return true;
        }
        if (onBottom) {
            *result = HTBOTTOM;
            return true;
        }
        if (onLeft) {
            *result = HTLEFT;
            return true;
        }
        if (onRight) {
            *result = HTRIGHT;
            return true;
        }

        if (titleBar && titleBar->isVisible()) {
            const QPoint winPos(static_cast<int>(x), static_cast<int>(y));
            const QPoint barTopLeft = titleBar->mapTo(window, QPoint(0, 0));
            const QRect barRect(barTopLeft, titleBar->size());
            if (barRect.contains(winPos)) {
                const QPoint barPos = titleBar->mapFrom(window, winPos);
                if (QWidget* child = titleBar->childAt(barPos)) {
                    if (child->inherits("QAbstractButton")) {
                        *result = HTCLIENT;
                        return true;
                    }
                }
                *result = HTCAPTION;
                return true;
            }
        }
    }
#endif

    return false;
}






