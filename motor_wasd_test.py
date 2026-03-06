#!/usr/bin/env python3
"""
motor_wasd_test.py
Yahboom YB-EJF01 VER1.4 확장보드 DC 모터 WASD 제어 테스트
PCA9685 (I2C 0x40) 2채널 H-Bridge 직접 제어

하드웨어 매뉴얼 기준 채널 매핑 (모터 1개당 2채널):
  Motor 1 (Right): CH8 (IN1A), CH9 (IN1B)
  Motor 2 (Left):  CH10 (IN2A), CH11 (IN2B)

TB6612FNG STBY 핀:
  라즈베리파이에서는 STBY 핀에 해당하는 GPIO를 수동으로
  HIGH로 설정해야 모터가 동작함 (Jetson Nano에서는 자동 처리)

조작법:
  W : 전진       S : 후진
  A : 좌회전     D : 우회전
  Q : 제자리 좌회전  E : 제자리 우회전
  + : 속도 증가   - : 속도 감소
  Space : 정지
  X / ESC : 종료
"""
import time
import curses
import smbus

# ─── STBY 핀 활성화 (RPi.GPIO) ───
# TB6612FNG STBY 핀이 어느 GPIO에 연결되어 있는지 모르므로
# 후보 GPIO를 모두 HIGH로 설정
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# I2C(2,3), UART(14,15), 부저(6), LED(23,24), 버튼(7,8) 제외
STBY_SKIP = {2, 3, 6, 7, 8, 14, 15, 23, 24}
STBY_CANDIDATES = [p for p in range(4, 28) if p not in STBY_SKIP]

def enable_stby():
    """모든 후보 GPIO를 HIGH로 → STBY 핀 활성화"""
    for pin in STBY_CANDIDATES:
        try:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)
        except Exception:
            pass

def disable_stby():
    """GPIO 정리"""
    GPIO.cleanup()


# ─── PCA9685 설정 ───
PCA9685_ADDR = 0x40
MODE1        = 0x00
PRESCALE     = 0xFE

def led_on_l(ch):  return 0x06 + 4 * ch
def led_on_h(ch):  return 0x07 + 4 * ch
def led_off_l(ch): return 0x08 + 4 * ch
def led_off_h(ch): return 0x09 + 4 * ch

# ─── 하드웨어 매뉴얼 기준 채널 매핑 (2채널 방식) ───
# Motor 1 (오른쪽): CH8 (IN1A), CH9 (IN1B)
# Motor 2 (왼쪽):   CH10 (IN2A), CH11 (IN2B)
MOTOR_R_A = 9   # IN1B (A/B 교체 - 방향 보정)
MOTOR_R_B = 8   # IN1A

MOTOR_L_A = 10  # IN2A
MOTOR_L_B = 11  # IN2B

bus = smbus.SMBus(1)


def pca9685_init():
    """PCA9685 초기화, PWM 주파수 ~1000Hz"""
    bus.write_byte_data(PCA9685_ADDR, MODE1, 0x00)
    time.sleep(0.005)
    old_mode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    bus.write_byte_data(PCA9685_ADDR, MODE1, (old_mode & 0x7F) | 0x10)  # sleep
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, 5)  # ~1000Hz
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode)  # wake
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode | 0x80)  # restart


def set_pwm(channel, value):
    """PCA9685 채널에 PWM 값 설정 (0~4095)"""
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


def motor_run(ch_a, ch_b, speed):
    """
    모터 구동 (2채널 직접 제어)
    speed > 0 : 정방향 (CH_A=속도, CH_B=0)
    speed < 0 : 역방향 (CH_A=0, CH_B=속도)
    speed = 0 : 정지
    """
    if speed > 0:
        set_pwm(ch_a, speed)
        set_pwm(ch_b, 0)
    elif speed < 0:
        set_pwm(ch_a, 0)
        set_pwm(ch_b, -speed)
    else:
        set_pwm(ch_a, 0)
        set_pwm(ch_b, 0)


def motor_right(speed):
    motor_run(MOTOR_R_A, MOTOR_R_B, speed)

def motor_left(speed):
    motor_run(MOTOR_L_A, MOTOR_L_B, speed)

def all_stop():
    motor_right(0)
    motor_left(0)


# ─── 이동 함수 ───
def go_forward(speed):
    motor_right(speed)
    motor_left(speed)

def go_backward(speed):
    motor_right(-speed)
    motor_left(-speed)

def turn_left(speed):
    motor_right(speed)
    motor_left(0)

def turn_right(speed):
    motor_right(0)
    motor_left(speed)

def spin_left(speed):
    motor_right(speed)
    motor_left(-speed)

def spin_right(speed):
    motor_right(-speed)
    motor_left(speed)


def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    speed = 2000
    SPEED_STEP = 400
    state = "정지"

    pca9685_init()
    enable_stby()  # ★ STBY 활성화

    def draw_ui():
        stdscr.clear()
        stdscr.addstr(0,  0, "==========================================")
        stdscr.addstr(1,  0, "  DC Motor WASD Test (2CH + STBY FIX)")
        stdscr.addstr(2,  0, "==========================================")
        stdscr.addstr(3,  0, "")
        stdscr.addstr(4,  0, "  State: {}".format(state))
        stdscr.addstr(5,  0, "  Speed: {} / 4095  ({}%)".format(speed, speed * 100 // 4095))
        stdscr.addstr(6,  0, "  STBY: ALL GPIO HIGH ({} pins)".format(len(STBY_CANDIDATES)))
        stdscr.addstr(7,  0, "")
        stdscr.addstr(8,  0, "  --- Controls ---")
        stdscr.addstr(9,  0, "  W : Forward     S : Backward")
        stdscr.addstr(10, 0, "  A : Turn Left   D : Turn Right")
        stdscr.addstr(11, 0, "  Q : Spin Left   E : Spin Right")
        stdscr.addstr(12, 0, "  + : Speed Up    - : Speed Down")
        stdscr.addstr(13, 0, "  Space : Stop")
        stdscr.addstr(14, 0, "  X / ESC : Quit")
        stdscr.addstr(15, 0, "==========================================")
        stdscr.addstr(16, 0, "  R_Motor: A=CH{} B=CH{}".format(MOTOR_R_A, MOTOR_R_B))
        stdscr.addstr(17, 0, "  L_Motor: A=CH{} B=CH{}".format(MOTOR_L_A, MOTOR_L_B))
        stdscr.refresh()

    draw_ui()

    while True:
        key = stdscr.getch()
        if key == -1:
            continue

        ch = chr(key).lower() if 0 <= key < 256 else ''

        if ch == 'w':
            go_forward(speed)
            state = ">> Forward"
        elif ch == 's':
            go_backward(speed)
            state = "<< Backward"
        elif ch == 'a':
            turn_left(speed)
            state = "<  Turn Left"
        elif ch == 'd':
            turn_right(speed)
            state = " > Turn Right"
        elif ch == 'q':
            spin_left(speed)
            state = "<< Spin Left"
        elif ch == 'e':
            spin_right(speed)
            state = ">> Spin Right"
        elif ch == ' ':
            all_stop()
            state = "[] Stopped"
        elif ch in ('+', '='):
            speed = min(4095, speed + SPEED_STEP)
            state = "Speed UP -> {}".format(speed)
        elif ch in ('-', '_'):
            speed = max(0, speed - SPEED_STEP)
            state = "Speed DOWN -> {}".format(speed)
        elif ch == 'x' or key == 27:
            all_stop()
            break

        draw_ui()

    stdscr.addstr(19, 0, "  Exited. Motors stopped.")
    stdscr.refresh()
    time.sleep(1)


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        all_stop()
        disable_stby()
        bus.close()
        print("Motors stopped. GPIO cleaned. Exit.")
