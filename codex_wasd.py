import time
import sys
import termios
import tty
import select

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

# Yahboom JetBot Hardware Manual 기준 매핑
I2C_ADDR = 0x40
FREQ_HZ = 1600

RIGHT_A = 8
RIGHT_B = 9
LEFT_A = 10
LEFT_B = 11

# 속도/턴 조정
SPEED = 0.6       # 전진/후진 기본 속도 (0.0~1.0)
TURN_SCALE = 0.5  # 회전 시 한쪽 바퀴 속도 배율
IDLE_STOP_SEC = 0.2  # 이 시간 동안 입력 없으면 정지

def make_motor(pca, a, b):
    # 방향이 반대면 a/b를 바꾸세요.
    return motor.DCMotor(pca.channels[a], pca.channels[b])

def stop(m, brake=True):
    # brake=True면 강제 브레이크(0), False면 코스트(None)
    m.throttle = 0 if brake else None

def set_motion(left, right, lspd, rspd):
    left.throttle = lspd
    right.throttle = rspd

def read_key(timeout=0.05):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c, address=I2C_ADDR)
    pca.frequency = FREQ_HZ

    right = make_motor(pca, RIGHT_A, RIGHT_B)
    left = make_motor(pca, LEFT_A, LEFT_B)

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    last_input = time.monotonic()
    try:
        print("WASD 조작 시작 (q: 종료). 입력 없으면 정지.")
        while True:
            key = read_key(0.05)
            now = time.monotonic()

            if key:
                last_input = now
                k = key.lower()

                if k == 'w':
                    set_motion(left, right, SPEED, SPEED)
                elif k == 's':
                    set_motion(left, right, -SPEED, -SPEED)
                elif k == 'a':
                    set_motion(left, right, SPEED * TURN_SCALE, SPEED)
                elif k == 'd':
                    set_motion(left, right, SPEED, SPEED * TURN_SCALE)
                elif k == ' ':
                    stop(left, brake=True)
                    stop(right, brake=True)
                elif k == 'q':
                    break

            if now - last_input > IDLE_STOP_SEC:
                stop(left, brake=True)
                stop(right, brake=True)

    finally:
        stop(left, brake=True)
        stop(right, brake=True)
        pca.deinit()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

if __name__ == "__main__":
    main()
