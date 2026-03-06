#!/usr/bin/env python3
"""
motor_stby_finder.py
TB6612FNG STBY 핀 찾기 + 모터 구동 진단 도구

GPIO를 하나씩 HIGH로 설정하면서 Adafruit_MotorHAT으로 모터를 돌려봄.
특정 GPIO를 HIGH로 했을 때 비로소 바퀴가 돌아가면 그게 STBY 핀.
"""
import time
import sys
import RPi.GPIO as GPIO

try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT
except ImportError:
    print("Adafruit_MotorHAT 필요: sudo pip3 install Adafruit-MotorHAT --break-system-packages")
    sys.exit(1)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

mh = Adafruit_MotorHAT(addr=0x40, i2c_bus=1)

def stop_all():
    for i in range(1, 5):
        mh.getMotor(i).run(Adafruit_MotorHAT.RELEASE)

def try_motors(speed=200):
    """Motor 1~4 동시에 FORWARD 시도"""
    for i in range(1, 5):
        m = mh.getMotor(i)
        m.setSpeed(speed)
        m.run(Adafruit_MotorHAT.FORWARD)

# I2C(SDA=2,SCL=3), UART(14,15), 서보 시리얼 등 사용중인 핀 제외
# 나머지 GPIO를 후보로 테스트
SKIP_PINS = {2, 3, 14, 15}  # I2C, UART는 건너뜀
CANDIDATE_PINS = [pin for pin in range(4, 28) if pin not in SKIP_PINS]

print("=" * 50)
print("  TB6612FNG STBY 핀 찾기 도구")
print("=" * 50)
print("")
print("방법: GPIO를 하나씩 HIGH로 설정하면서")
print("      Motor 1~4를 동시에 FORWARD로 돌림")
print("      바퀴가 돌아가면 그 GPIO가 STBY 핀!")
print("")
print("테스트할 GPIO:", CANDIDATE_PINS)
print("")

# ─── 모드 1: 자동 스캔 ───
def auto_scan():
    print("=== 자동 스캔 모드 ===")
    print("각 GPIO를 3초간 HIGH로 설정하고 모터를 돌립니다.")
    print("바퀴가 돌아가는 순간 Ctrl+C를 누르세요!\n")

    activated_pins = []

    try:
        for pin in CANDIDATE_PINS:
            # 이전 핀 정리
            for p in activated_pins:
                GPIO.setup(p, GPIO.IN)
            activated_pins.clear()

            # 현재 핀 HIGH
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
            activated_pins.append(pin)

            # 모터 구동
            try_motors(200)

            print("  GPIO {} = HIGH, Motor ON... (3초 대기)".format(pin), end="", flush=True)
            time.sleep(3)

            stop_all()
            print(" -> 바퀴 안 돌아감" if True else "")  

        print("\n모든 GPIO 단일 테스트 완료. 바퀴가 돌아간 핀이 없었다면 모드2를 시도하세요.")

    except KeyboardInterrupt:
        stop_all()
        print("\n\n>>> GPIO {} 에서 바퀴가 돌아갔나요?! <<<".format(pin))
        print("    그렇다면 이 GPIO가 STBY 핀입니다!")

    finally:
        stop_all()
        for p in activated_pins:
            GPIO.setup(p, GPIO.IN)

# ─── 모드 2: 전체 GPIO HIGH + 모터 테스트 ───
def all_high_test():
    print("\n=== 전체 GPIO HIGH 테스트 ===")
    print("모든 후보 GPIO를 HIGH로 설정하고 모터를 돌립니다.")
    print("이 상태에서 바퀴가 돌아가면 STBY 핀이 후보 중에 있는 것!\n")

    for pin in CANDIDATE_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)
    print("  {} 핀 모두 HIGH 설정 완료".format(len(CANDIDATE_PINS)))

    try_motors(200)
    input("  Motor ON! 바퀴가 돌아가나요? [Enter로 정지]")

    stop_all()
    for pin in CANDIDATE_PINS:
        GPIO.setup(pin, GPIO.IN)
    print("  정지 및 GPIO 복원 완료")

# ─── 모드 3: 수동 GPIO 지정 + 모터 테스트 ───
def manual_test():
    print("\n=== 수동 GPIO 지정 테스트 ===")
    try:
        pin = int(input("  HIGH로 설정할 GPIO 번호 (BCM): "))
    except ValueError:
        print("  잘못된 입력")
        return

    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)
    print("  GPIO {} = HIGH".format(pin))

    try_motors(200)
    input("  Motor ON! 바퀴가 돌아가나요? [Enter로 정지]")

    stop_all()
    GPIO.setup(pin, GPIO.IN)
    print("  정지 완료")


if __name__ == "__main__":
    try:
        print("모드 선택:")
        print("  1: 자동 스캔 (GPIO 하나씩 HIGH → 모터 테스트)")
        print("  2: 전체 GPIO HIGH → 모터 테스트 (먼저 이걸로 STBY 존재 여부 확인)")
        print("  3: 수동 GPIO 지정 테스트")
        print("  q: 종료")
        print("")

        while True:
            choice = input("선택> ").strip().lower()
            if choice == '1':
                auto_scan()
            elif choice == '2':
                all_high_test()
            elif choice == '3':
                manual_test()
            elif choice == 'q':
                break
            else:
                print("1, 2, 3, q 중 선택하세요.")

    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        stop_all()
        GPIO.cleanup()
        print("GPIO 정리 완료, 종료")
