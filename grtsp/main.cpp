#include <QApplication>
#include <QLabel>
#include <QGridLayout>
#include "camera_system.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    
    // UI 설정
    QWidget window;
    QGridLayout* layout = new QGridLayout(&window);
    
    QLabel* labels[6];
    for (int i = 0; i < 6; i++) {
        labels[i] = new QLabel();
        labels[i]->setMinimumSize(640, 480);
        labels[i]->setScaledContents(true);
        layout->addWidget(labels[i], i / 3, i % 3);
    }
    
    // 카메라 매니저 시작
    CameraManager manager;
    
    QObject::connect(&manager, &CameraManager::cameraFrameReady,
        [&labels](int cameraId, QImage frame) {
            if (cameraId >= 0 && cameraId < 6) {
                labels[cameraId]->setPixmap(QPixmap::fromImage(frame));
            }
        }
    );
    
    if (!manager.startAll()) {
        qCritical() << "카메라 시작 실패";
        return 1;
    }
    
    window.setWindowTitle("6채널 카메라 모니터링");
    window.resize(1920, 1080);
    window.show();
    
    return app.exec();
}