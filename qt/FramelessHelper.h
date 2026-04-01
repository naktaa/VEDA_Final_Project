#ifndef FRAMELESSHELPER_H
#define FRAMELESSHELPER_H

#include <QByteArray>

class QWidget;

class FramelessHelper
{
public:
    static bool handleNativeEvent(QWidget* window, QWidget* titleBar,
                                  const QByteArray& eventType,
                                  void* message, qintptr* result);
};

#endif // FRAMELESSHELPER_H
