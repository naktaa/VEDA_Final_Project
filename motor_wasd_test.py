#!/usr/bin/env python3
"""
motor_wasd_test.py
Yahboom YB-EJF01 VER1.4 확장보드 DC 모터 WASD 제어 테스트
PCA9685 (I2C 0x40) + TB6612FNG H-Bridge 로 DC 모터 2개 제어

TB6612FNG 제어 방식 (모터 1개당 3채널):
  PWM 채널 : 속도 제어 (0~4095)
  IN1 채널 : 방향 제어 핀 1 (0 또는 4095)
  IN2 채널 : 방향 제어 핀 2 (0 또는 4095)

  전진: IN1=HIGH, IN2=LOW,  PWM=속도
  후진: IN1=LOW,  IN2=HIGH, PWM=속도
  정지: IN1=LOW,  IN2=LOW,  PWM=0

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

# ─── PCA9685 설정 ───
PCA9685_ADDR = 0x40
MODE1        = 0x00
PRESCALE     = 0xFE

def led_on_l(ch):  return 0x06 + 4 * ch
def led_on_h(ch):  return 0x07 + 4 * ch
def led_off_l(ch): return 0x08 + 4 * ch
def led_off_h(ch): return 0x09 + 4 * ch

# ─── Adafruit MotorHAT 호환 채널 매핑 (TB6612FNG) ───
# Motor 1 (오른쪽): PWM=CH8,  IN2=CH9,  IN1=CH10
# Motor 2 (왼쪽):   PWM=CH13, IN2=CH12, IN1=CH11
MOTOR_R_PWM = 8
MOTOR_R_IN2 = 9
MOTOR_R_IN1 = 10

MOTOR_L_PWM = 13
MOTOR_L_IN2 = 12
MOTOR_L_IN1 = 11

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
        # 완전 OFF
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0)
    elif value >= 4095:
        # 완전 ON (full on bit 설정)
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0x10)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0)
    else:
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), value & 0xFF)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), (value >> 8) & 0x0F)


def motor_run(pwm_ch, in1_ch, in2_ch, speed):
    """
    모터 구동 (TB6612FNG 방식)
    speed > 0 : 정방향 (IN1=HIGH, IN2=LOW)
    speed < 0 : 역방향 (IN1=LOW, IN2=HIGH)
    speed = 0 : 정지
    """
    if speed > 0:
        set_pwm(in1_ch, 4095)   # IN1 = HIGH
        set_pwm(in2_ch, 0)      # IN2 = LOW
        set_pwm(pwm_ch, speed)  # PWM = 속도
    elif speed < 0:
        set_pwm(in1_ch, 0)      # IN1 = LOW
        set_pwm(in2_ch, 4095)   # IN2 = HIGH
        set_pwm(pwm_ch, -speed) # PWM = 속도 (절대값)
    else:
        set_pwm(in1_ch, 0)      # IN1 = LOW
        set_pwm(in2_ch, 0)      # IN2 = LOW
        set_pwm(pwm_ch, 0)      # PWM = 0


def motor_right(speed):
    """오른쪽 모터 제어 (양수=전진, 음수=후진)"""
    motor_run(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, speed)

def motor_left(speed):
    """왼쪽 모터 제어 (양수=전진, 음수=후진)"""
    motor_run(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, speed)

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
    """좌회전: 오른쪽만 전진"""
    motor_right(speed)
    motor_left(0)

def turn_right(speed):
    """우회전: 왼쪽만 전진"""
    motor_right(0)
    motor_left(speed)

def spin_left(speed):
    """제자리 좌회전"""
    motor_right(speed)
    motor_left(-speed)

def spin_right(speed):
    """제자리 우회전"""
    motor_right(-speed)
    motor_left(speed)


def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    speed = 2000              # 초기 속도 (0~4095)
    SPEED_STEP = 400
    state = "정지"

    pca9685_init()

    def draw_ui():
        stdscr.clear()
        stdscr.addstr(0,  0, "==========================================")
        stdscr.addstr(1,  0, "  Yahboom DC Motor WASD Test (TB6612FNG)")
        stdscr.addstr(2,  0, "==========================================")
        stdscr.addstr(3,  0, "")
        stdscr.addstr(4,  0, "  State: {}".format(state))
        stdscr.addstr(5,  0, "  Speed: {} / 4095  ({}%)".format(speed, speed * 100 // 4095))
        stdscr.addstr(6,  0, "")
        stdscr.addstr(7,  0, "  --- Controls ---")
        stdscr.addstr(8,  0, "  W : Forward     S : Backward")
        stdscr.addstr(9,  0, "  A : Turn Left   D : Turn Right")
        stdscr.addstr(10, 0, "  Q : Spin Left   E : Spin Right")
        stdscr.addstr(11, 0, "  + : Speed Up    - : Speed Down")
        stdscr.addstr(12, 0, "  Space : Stop")
        stdscr.addstr(13, 0, "  X / ESC : Quit")
        stdscr.addstr(14, 0, "==========================================")
        stdscr.addstr(15, 0, "  R_Motor: PWM=CH{} IN1=CH{} IN2=CH{}".format(
            MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2))
        stdscr.addstr(16, 0, "  L_Motor: PWM=CH{} IN1=CH{} IN2=CH{}".format(
            MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2))
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

    stdscr.addstr(18, 0, "  Exited. Motors stopped.")
    stdscr.refresh()
    time.sleep(1)


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        all_stop()
        bus.close()
        print("Motors stopped. Program exit.")
