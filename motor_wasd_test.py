#!/usr/bin/env python3
"""
motor_wasd_test.py
Yahboom YB-EJF01 VER1.4 확장보드 DC 모터 WASD 제어 테스트
PCA9685 (I2C 0x40) + H-Bridge 로 DC 모터 2개 제어

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
import smbus2

# ─── PCA9685 설정 ───
PCA9685_ADDR = 0x40
MODE1        = 0x00
PRESCALE     = 0xFE

def led_on_l(ch):  return 0x06 + 4 * ch
def led_on_h(ch):  return 0x07 + 4 * ch
def led_off_l(ch): return 0x08 + 4 * ch
def led_off_h(ch): return 0x09 + 4 * ch

# ─── 모터 채널 매핑 (Yahboom 기본값) ───
MOTOR_R_IN_A = 8    # 오른쪽 모터 정방향
MOTOR_R_IN_B = 9    # 오른쪽 모터 역방향
MOTOR_L_IN_A = 10   # 왼쪽 모터 정방향
MOTOR_L_IN_B = 11   # 왼쪽 모터 역방향

bus = smbus2.SMBus(1)


def pca9685_init():
    """PCA9685 초기화, PWM 주파수 ~1000Hz"""
    bus.write_byte_data(PCA9685_ADDR, MODE1, 0x00)
    time.sleep(0.005)
    old_mode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    bus.write_byte_data(PCA9685_ADDR, MODE1, (old_mode & 0x7F) | 0x10)
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, 5)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode | 0x80)


def set_pwm(channel, value):
    """PCA9685 채널에 PWM 값 설정 (0~4095)"""
    if value <= 0:
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


def motor_forward(in_a, in_b, speed):
    set_pwm(in_a, speed)
    set_pwm(in_b, 0)

def motor_backward(in_a, in_b, speed):
    set_pwm(in_a, 0)
    set_pwm(in_b, speed)

def motor_stop(in_a, in_b):
    set_pwm(in_a, 0)
    set_pwm(in_b, 0)

def all_stop():
    motor_stop(MOTOR_R_IN_A, MOTOR_R_IN_B)
    motor_stop(MOTOR_L_IN_A, MOTOR_L_IN_B)


# ─── 이동 함수 ───
def go_forward(speed):
    motor_forward(MOTOR_R_IN_A, MOTOR_R_IN_B, speed)
    motor_forward(MOTOR_L_IN_A, MOTOR_L_IN_B, speed)

def go_backward(speed):
    motor_backward(MOTOR_R_IN_A, MOTOR_R_IN_B, speed)
    motor_backward(MOTOR_L_IN_A, MOTOR_L_IN_B, speed)

def turn_left(speed):
    """좌회전: 오른쪽만 전진"""
    motor_forward(MOTOR_R_IN_A, MOTOR_R_IN_B, speed)
    motor_stop(MOTOR_L_IN_A, MOTOR_L_IN_B)

def turn_right(speed):
    """우회전: 왼쪽만 전진"""
    motor_stop(MOTOR_R_IN_A, MOTOR_R_IN_B)
    motor_forward(MOTOR_L_IN_A, MOTOR_L_IN_B, speed)

def spin_left(speed):
    """제자리 좌회전: 오른쪽 전진 + 왼쪽 후진"""
    motor_forward(MOTOR_R_IN_A, MOTOR_R_IN_B, speed)
    motor_backward(MOTOR_L_IN_A, MOTOR_L_IN_B, speed)

def spin_right(speed):
    """제자리 우회전: 오른쪽 후진 + 왼쪽 전진"""
    motor_backward(MOTOR_R_IN_A, MOTOR_R_IN_B, speed)
    motor_forward(MOTOR_L_IN_A, MOTOR_L_IN_B, speed)


def main(stdscr):
    curses.curs_set(0)            # 커서 숨기기
    stdscr.nodelay(True)          # 비블로킹 입력
    stdscr.timeout(100)           # 100ms마다 갱신

    speed = 2000                  # 초기 속도 (0~4095)
    SPEED_STEP = 400              # 속도 증감 단위
    state = "정지"

    pca9685_init()

    # ─── UI 그리기 ───
    def draw_ui():
        stdscr.clear()
        stdscr.addstr(0, 0, "═══════════════════════════════════════")
        stdscr.addstr(1, 0, "  Yahboom DC Motor WASD 테스트")
        stdscr.addstr(2, 0, "═══════════════════════════════════════")
        stdscr.addstr(3, 0, "")
        stdscr.addstr(4, 0, f"  상태: {state}")
        stdscr.addstr(5, 0, f"  속도: {speed} / 4095  ({speed*100//4095}%)")
        stdscr.addstr(6, 0, "")
        stdscr.addstr(7, 0, "  ─── 조작법 ───")
        stdscr.addstr(8, 0, "  W : 전진        S : 후진")
        stdscr.addstr(9, 0, "  A : 좌회전      D : 우회전")
        stdscr.addstr(10, 0, "  Q : 제자리좌회전  E : 제자리우회전")
        stdscr.addstr(11, 0, "  + : 속도 증가    - : 속도 감소")
        stdscr.addstr(12, 0, "  Space : 정지")
        stdscr.addstr(13, 0, "  X / ESC : 종료")
        stdscr.addstr(14, 0, "═══════════════════════════════════════")
        stdscr.refresh()

    draw_ui()

    while True:
        key = stdscr.getch()
        if key == -1:
            continue

        ch = chr(key).lower() if 0 <= key < 256 else ''

        if ch == 'w':
            go_forward(speed)
            state = "▲ 전진"
        elif ch == 's':
            go_backward(speed)
            state = "▼ 후진"
        elif ch == 'a':
            turn_left(speed)
            state = "◄ 좌회전"
        elif ch == 'd':
            turn_right(speed)
            state = "► 우회전"
        elif ch == 'q':
            spin_left(speed)
            state = "↺ 제자리 좌회전"
        elif ch == 'e':
            spin_right(speed)
            state = "↻ 제자리 우회전"
        elif ch == ' ':
            all_stop()
            state = "■ 정지"
        elif ch in ('+', '='):
            speed = min(4095, speed + SPEED_STEP)
            state = f"속도 증가 → {speed}"
        elif ch in ('-', '_'):
            speed = max(0, speed - SPEED_STEP)
            state = f"속도 감소 → {speed}"
            # 현재 움직이는 중이면 동적으로 반영되지 않음 (다음 키 입력시 적용)
        elif ch == 'x' or key == 27:  # x 또는 ESC
            all_stop()
            break

        draw_ui()

    stdscr.addstr(16, 0, "  종료됨. 모터 정지 완료.")
    stdscr.refresh()
    time.sleep(1)


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        all_stop()
        bus.close()
        print("모터 정지, 프로그램 종료")
