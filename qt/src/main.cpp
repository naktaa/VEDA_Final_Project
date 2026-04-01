#include <QApplication>
#include <QDialog>
#include <gst/gst.h>
#include "login_dialog.h"
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);   // ✅ 추가
    QApplication app(argc, argv);

    // 1) 로그인 다이얼로그 실행
    LoginDialog login;
    if (login.exec() != QDialog::Accepted) {
        return 0; // 로그인 취소/실패면 종료
    }

    // 2) 로그인 성공 -> userId 전달 -> MainWindow에서 role 결정
    MainWindow w(login.userId());
    w.showMaximized();

    return app.exec();
}
