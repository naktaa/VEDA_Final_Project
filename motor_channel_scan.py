#!/usr/bin/env python3
"""
motor_channel_scan.py
PCA9685 채널 스캔 - 어떤 채널이 어떤 모터에 연결되어 있는지 확인하는 도구

CH0 ~ CH15를 하나씩 HIGH(4095)로 켜보면서
바퀴가 움직이거나 소리가 나는 채널을 기록하세요.
"""
import time
import smbus

PCA9685_ADDR = 0x40
MODE1        = 0x00
PRESCALE     = 0xFE

bus = smbus.SMBus(1)

def led_on_l(ch):  return 0x06 + 4 * ch
def led_on_h(ch):  return 0x07 + 4 * ch
def led_off_l(ch): return 0x08 + 4 * ch
def led_off_h(ch): return 0x09 + 4 * ch

def pca9685_init():
    bus.write_byte_data(PCA9685_ADDR, MODE1, 0x00)
    time.sleep(0.005)
    old_mode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    bus.write_byte_data(PCA9685_ADDR, MODE1, (old_mode & 0x7F) | 0x10)
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, 5)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode | 0x80)

def set_pwm(channel, value):
    value = max(0, min(4095, value))
    if value == 0:
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0)
    elif value >= 4095:
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0x10)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0)
    else:
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), value & 0xFF)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), (value >> 8) & 0x0F)

def all_off():
    for ch in range(16):
        set_pwm(ch, 0)

# ─── 모드 1: 단일 채널 스캔 ───
def scan_single():
    print("\n=== 모드 1: 단일 채널 스캔 ===")
    print("CH0~CH15를 하나씩 HIGH(4095)로 켭니다.")
    print("각 채널에서 바퀴 반응을 관찰하세요.\n")
    for ch in range(16):
        input("  [Enter] 를 누르면 CH{} 을 HIGH 로 켭니다...".format(ch))
        all_off()
        set_pwm(ch, 4095)
        print("    --> CH{} = HIGH (4095)  관찰하세요!".format(ch))
    all_off()
    print("\n단일 채널 스캔 완료. 모든 채널 OFF.\n")

# ─── 모드 2: 3채널 조합 테스트 ───
def scan_combo():
    print("\n=== 모드 2: 3채널 조합 테스트 ===")
    print("Adafruit MotorHAT 표준 매핑 4가지를 테스트합니다.\n")

    combos = [
        ("Motor1", 8, 10, 9),   # PWM=8,  IN1=10, IN2=9
        ("Motor2", 13, 11, 12), # PWM=13, IN1=11, IN2=12
        ("Motor3", 2, 4, 3),    # PWM=2,  IN1=4,  IN2=3
        ("Motor4", 7, 5, 6),    # PWM=7,  IN1=5,  IN2=6
    ]

    for name, pwm, in1, in2 in combos:
        input("  [Enter] -> {} 전진 테스트 (PWM=CH{}, IN1=CH{}, IN2=CH{})...".format(
            name, pwm, in1, in2))
        all_off()
        set_pwm(in1, 4095)    # IN1 = HIGH
        set_pwm(in2, 0)       # IN2 = LOW
        set_pwm(pwm, 2000)    # PWM = 속도
        print("    --> {} 가동 중! 바퀴가 움직이나요?".format(name))

        input("  [Enter] -> {} 정지...".format(name))
        all_off()
        print("    --> {} 정지\n".format(name))

    print("3채널 조합 테스트 완료.\n")

# ─── 모드 3: 사용자 지정 채널 테스트 ───
def scan_custom():
    print("\n=== 모드 3: 사용자 지정 3채널 테스트 ===")
    print("원하는 PWM, IN1, IN2 채널 번호를 입력하세요.\n")

    try:
        pwm = int(input("  PWM 채널 (0~15): "))
        in1 = int(input("  IN1 채널 (0~15): "))
        in2 = int(input("  IN2 채널 (0~15): "))
    except ValueError:
        print("  잘못된 입력!")
        return

    print("\n  전진 테스트: PWM=CH{}, IN1=CH{}=HIGH, IN2=CH{}=LOW".format(pwm, in1, in2))
    all_off()
    set_pwm(in1, 4095)
    set_pwm(in2, 0)
    set_pwm(pwm, 2000)

    input("  [Enter] -> 정지")
    all_off()

    print("  역방향 테스트: PWM=CH{}, IN1=CH{}=LOW, IN2=CH{}=HIGH".format(pwm, in1, in2))
    set_pwm(in1, 0)
    set_pwm(in2, 4095)
    set_pwm(pwm, 2000)

    input("  [Enter] -> 정지")
    all_off()
    print("  테스트 완료.\n")


if __name__ == "__main__":
    try:
        pca9685_init()
        print("PCA9685 초기화 OK (I2C 0x40)")
        print("")
        print("=== PCA9685 채널 스캔 도구 ===")
        print("  1: 단일 채널 스캔 (CH0~15 하나씩 HIGH)")
        print("  2: 3채널 조합 테스트 (Adafruit MotorHAT 표준 4가지)")
        print("  3: 사용자 지정 채널 테스트")
        print("  q: 종료")
        print("")

        while True:
            choice = input("선택> ").strip().lower()
            if choice == '1':
                scan_single()
            elif choice == '2':
                scan_combo()
            elif choice == '3':
                scan_custom()
            elif choice == 'q':
                break
            else:
                print("1, 2, 3, q 중 선택하세요.")

    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        all_off()
        bus.close()
        print("모든 채널 OFF, 종료")
