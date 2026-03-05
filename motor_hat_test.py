#!/usr/bin/env python3
"""
motor_hat_test.py
Yahboom YB-EJF01 확장보드 DC 모터 테스트
Adafruit_MotorHAT 공식 라이브러리 사용 (Yahboom 공식 방식)

설치:
  sudo pip3 install Adafruit-MotorHAT --break-system-packages
  (또는)
  sudo apt install python3-pip
  pip3 install Adafruit-MotorHAT --break-system-packages
"""
import time
import sys

try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT
except ImportError:
    print("Adafruit_MotorHAT 라이브러리가 없습니다!")
    print("설치: sudo pip3 install Adafruit-MotorHAT --break-system-packages")
    sys.exit(1)

# ─── MotorHAT 초기화 (I2C 주소 0x40) ───
mh = Adafruit_MotorHAT(addr=0x40, i2c_bus=1)

# Motor 1, 2, 3, 4 중 어떤 것이 연결되어 있는지 테스트
def test_motor(motor_num, speed=150):
    """지정 모터 번호(1~4)를 전진/후진 테스트"""
    print("\n--- Motor {} 테스트 ---".format(motor_num))
    motor = mh.getMotor(motor_num)

    # 속도 설정 (0~255)
    motor.setSpeed(speed)

    input("  [Enter] -> Motor {} 정방향 (FORWARD)".format(motor_num))
    motor.run(Adafruit_MotorHAT.FORWARD)
    time.sleep(2)

    motor.run(Adafruit_MotorHAT.RELEASE)
    time.sleep(0.5)

    input("  [Enter] -> Motor {} 역방향 (BACKWARD)".format(motor_num))
    motor.run(Adafruit_MotorHAT.BACKWARD)
    time.sleep(2)

    motor.run(Adafruit_MotorHAT.RELEASE)
    print("  Motor {} 정지".format(motor_num))


def stop_all():
    """모든 모터 정지"""
    for i in range(1, 5):
        mh.getMotor(i).run(Adafruit_MotorHAT.RELEASE)


if __name__ == "__main__":
    try:
        print("=" * 45)
        print("  Yahboom DC Motor Test (Adafruit_MotorHAT)")
        print("=" * 45)
        print("")
        print("Motor 1~4를 하나씩 테스트합니다.")
        print("어떤 번호에서 바퀴가 돌아가는지 확인하세요!")
        print("")

        for m in [1, 2, 3, 4]:
            test_motor(m, speed=200)

        print("\n모든 모터 테스트 완료!")

    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        stop_all()
        print("모터 정지, 종료")
