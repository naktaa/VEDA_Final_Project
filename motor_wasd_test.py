#!/usr/bin/env python3
"""
motor_wasd_test.py
Yahboom YB-EJF01 VER1.4 확장보드 DC 모터 WASD 제어 테스트
PCA9685 (I2C 0x40) 2채널 H-Bridge 직접 제어

하드웨어 매뉴얼 기준 채널 매핑 (모터 1개당 2채널):
  Motor 1 (Right): CH8 (IN1A), CH9 (IN1B)
  Motor 2 (Left):  CH10 (IN2A), CH11 (IN2B)

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

# ─── 하드웨어 매뉴얼 기준 채널 매핑 (2채널 방식) ───
# Motor Right: CH8 (IN1A), CH9 (IN1B)
# Motor Left : CH12 (IN3A), CH13 (IN3B)  # 10/11 ?? ?? ???? ??
MOTOR_R_A = 8   # IN1A
MOTOR_R_B = 9   # IN1B

MOTOR_L_A = 10  # IN3A
MOTOR_L_B = 11  # IN3B
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
        # ★ Full OFF 비트(bit4) 설정 → 출력 완전히 LOW
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0x10)
    elif value >= 4095:
        # ★ Full ON 비트(bit4) 설정 → 출력 완전히 HIGH
        # Full OFF 비트 해제 필요 (OFF가 ON보다 우선이므로)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0x10)
    else:
        # 일반 PWM: OFF 비트 먼저 해제 후 값 설정
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), value & 0xFF)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), (value >> 8) & 0x0F)


def all_off():
    """모든 모터 채널 완전 OFF"""
    for ch in [8, 9, 10, 11, 12, 13]:
        set_pwm(ch, 0)


def motor_run(ch_a, ch_b, speed):
    """
    모터 구동 (2채널 직접 제어)
    speed > 0 : 정방향 (CH_A=속도, CH_B=OFF)
    speed < 0 : 역방향 (CH_A=OFF, CH_B=속도)
    speed = 0 : 정지
    """
    if speed > 0:
        set_pwm(ch_b, 0)       # ★ 반대쪽 먼저 OFF
        set_pwm(ch_a, speed)   # 그 다음 구동
    elif speed < 0:
        set_pwm(ch_a, 0)       # ★ 반대쪽 먼저 OFF
        set_pwm(ch_b, -speed)  # 그 다음 구동
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

    speed = 4095              # ★ 초기 속도 최대
    SPEED_STEP = 400
    state = "정지"

    pca9685_init()
    all_off()                 # 초기화 시 모든 채널 확실히 OFF

    def draw_ui():
        stdscr.clear()
        stdscr.addstr(0,  0, "==========================================")
        stdscr.addstr(1,  0, "  DC Motor WASD Test (2CH PCA9685)")
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
        stdscr.addstr(15, 0, "  R_Motor: A=CH{} B=CH{}".format(MOTOR_R_A, MOTOR_R_B))
        stdscr.addstr(16, 0, "  L_Motor: A=CH{} B=CH{}".format(MOTOR_L_A, MOTOR_L_B))
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
        all_off()
        bus.close()
        print("Motors stopped. Exit.")
