#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define SERVER_IP "192.168.0.197" // PC(Qt)의 IP 주소로 변경하세요
#define SERVER_PORT 8888
#define QUALITY 50 // JPEG 압축률 (UDP 패킷 사이즈 초과 방지)

int main() {
    // 1. GStreamer 파이프라인 설정 (RPi OS Bullseye/Bookworm 기준)
    // 640x480 해상도로 설정하여 데이터 크기 관리
    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! "
                           "videoconvert ! video/x-raw, format=BGR ! appsink";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "카메라를 열 수 없습니다." << std::endl;
        return -1;
    }

    // 2. UDP 소켓 생성
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr);

    cv::Mat frame;
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, QUALITY};

    std::cout << "영상 전송 시작..." << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 3. JPEG 인코딩 (압축)
        cv::imencode(".jpg", frame, buffer, params);

        // 4. UDP 전송 (데이터 크기가 65,507 bytes를 넘으면 전송 실패할 수 있음)
        if (buffer.size() < 65500) {
            sendto(sock, buffer.data(), buffer.size(), 0, 
                   (struct sockaddr*)&serverAddr, sizeof(serverAddr));
        } else {
            std::cerr << "프레임이 너무 큽니다: " << buffer.size() << " bytes" << std::endl;
        }

        // 약간의 딜레이 (네트워크 폭주 방지)
        usleep(1000); 
    }

    close(sock);
    return 0;
}