// #include <iostream>
// #include <fstream>
// #include <string>
// #include <unistd.h>
// #include <opencv2/opencv.hpp>
// #include <wiringPi.h>
// #include <softPwm.h>
#include "main.h"
#include "mqtt_client.h"

using namespace std;
using namespace cv;

#define STOP 0
#define FORWARD 1
#define BACKWARD 2

#define IN3 27
#define IN4 28
#define ENB 29 // PWM enable

int start_cmd;

std::string pipeline =
    "libcamerasrc ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink";

// [주의] Pi 5는 보통 pwmchip2를 사용합니다. ls /sys/class/pwm으로 확인하세요.
const string CHIP_PATH = "/sys/class/pwm/pwmchip0/";
const string PWM_PATH = CHIP_PATH + "pwm0/"; // BCM 18은 보통 pwm2 또는 pwm0

static void setPinConfig(int EN, int INA, int INB)
{
    pinMode(EN, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);

    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);

    // soft PWM range 0~255
    if (softPwmCreate(EN, 0, 255) != 0)
    {
        printf("softPwmCreate 실패\n");
    }
}

static void setMotorControl(int EN, int INA, int INB, int speed, int stat)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    // speed (PWM on ENB)
    softPwmWrite(EN, speed);

    // direction
    if (stat == FORWARD)
    {
        digitalWrite(INA, LOW);
        digitalWrite(INB, HIGH);
    }
    else if (stat == BACKWARD)
    {
        digitalWrite(INA, HIGH);
        digitalWrite(INB, LOW);
    }
    else
    { // STOP (coast)
        softPwmWrite(EN, 0);
        digitalWrite(INA, LOW);
        digitalWrite(INB, LOW);
    }
}

void sysfsWrite(string path, string value)
{
    ofstream ofs(path);
    if (ofs.is_open())
    {
        ofs << value;
        ofs.close();
    }
    else
    {
        cerr << "Error writing to: " << path << endl;
    }
}

void setupPWM()
{
    // 1. 채널 활성화 (이미 되어있을 수 있음)
    sysfsWrite(CHIP_PATH + "export", "0"); // BCM 18번에 대응하는 채널 번호
    usleep(200000);

    // 2. 주기 설정 (20ms = 20,000,000ns)
    sysfsWrite(PWM_PATH + "period", "20000000");

    // 3. 초기 중앙 위치 (1250us = 1,250,000ns)
    sysfsWrite(PWM_PATH + "duty_cycle", "1250000");

    // 4. 출력 시작
    sysfsWrite(PWM_PATH + "enable", "1");
    cout << "Hardware PWM started (20ms period, 1250us center)" << endl;
}

int main()
{
    int flag = 0;
    setupPWM();
    bool ok = mqtt_start_subscriber("192.168.0.60", 1883, "jiho",
                                    [](const std::string &topic, const std::string &payload)
                                    {
                                        std::cout << "[MQTT] " << topic << " : " << payload << std::endl;
                                        if (payload == "start")
                                            start_cmd = 1;
                                        else if (payload == "stop")
                                            start_cmd = 0;
                                        // 여기서 payload 보고 RC카 제어 함수 호출하면 됨
                                        // 예: if(payload=="L") turn_left();
                                    });
    if (!ok)
    {
        std::cerr << "MQTT start failed\n";
        return 1;
    }
    if (wiringPiSetup() == -1)
    {
        printf("wiringPiSetup 실패\n");
        return 1;
    }
    setPinConfig(ENB, IN3, IN4);
    int us;
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        cerr << "Failed to open camera\n";
        return 1;
    }

    Mat frame, roi, gray, binary;

    while (true)
    {
        try
        {

            cap >> frame;
            if (frame.empty())
                break;

            // ROI: (0,160)부터 640x80
            Rect rect(0, 160, 640, 80);
            roi = frame(rect);
            cvtColor(roi, gray, COLOR_BGR2GRAY);
            GaussianBlur(gray, gray, Size(5, 5), 0);
            threshold(gray, binary, 100, 255, THRESH_BINARY_INV);
            Moments m = moments(binary, true);
            int speed = 0;
            if (start_cmd == 1 && flag == 0)
            {
                speed = 120;
                cout << "start\n";
                flag = 1;
            }
            else if (start_cmd == 0 && flag == 1)
            {
                speed = 0;
                flag = 0;
                cout << "stop\n";
            }
            if (m.m00 > 0)
            {
                int cx = (int)(m.m10 / m.m00);

                int error = 320 - cx;

                // 라인 중심 표시
                circle(roi, Point(cx, 40), 5, Scalar(0, 0, 255), -1);

                // 조향각 계산: 90도 기준, error에 비례해서 +/- (너가 쓰던 식 유지)
                us = 1250 + error * 500 / 320;
                // 안전 클램핑 (사용자 실측 데이터 기준)
                if (us < 750)
                    us = 750;
                if (us > 1750)
                    us = 1750;

                // cout << "Cx: " << cx
                //      << " error: " << error
                //      << " us: " << us << "\n";
            }
            // int us = stoi(input);

            // sysfs는 ns 단위를 쓰므로 1000을 곱함
            long ns = (long)us * 1000;
            sysfsWrite(PWM_PATH + "duty_cycle", to_string(ns));
            setMotorControl(ENB, IN3, IN4, speed, FORWARD);
            // cout << "적용: " << us << " us (" << ns << " ns)" << endl;
            imshow("Original", frame);
            imshow("Binary", binary);
            if (waitKey(1) == 'q')
                break;
        }
        catch (...)
        {
            cout << "숫자 또는 q만 입력해줘.\n";
        }
    }

    // 종료 시 중지
    sysfsWrite(PWM_PATH + "enable", "0");
    cap.release();
    destroyAllWindows();
    mqtt_stop();
    return 0;
}
