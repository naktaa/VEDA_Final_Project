import time
import sys
import termios
import tty
import select

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

I2C_ADDR = 0x40
FREQ_HZ = 1000  # 필요하면 100 ~ 1600 범위로 바꿔 테스트

# Adafruit Motor HAT/FeatherWing 매핑
EN_A = 8
A1 = 9
A2 = 10

EN_B = 13
B1 = 11
B2 = 12

SPEED = 0.6
TURN_SCALE = 0.5
IDLE_STOP_SEC = 0.2

def read_key(timeout=0.05):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c, address=I2C_ADDR)
    pca.frequency = FREQ_HZ

    # Enable 채널 HIGH
    pca.channels[EN_A].duty_cycle = 0xFFFF
    pca.channels[EN_B].duty_cycle = 0xFFFF

    motor_a = motor.DCMotor(pca.channels[A1], pca.channels[A2])  # 오른쪽/왼쪽은 상황에 따라 바꿔도 됨
    motor_b = motor.DCMotor(pca.channels[B1], pca.channels[B2])

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
                    motor_a.throttle = SPEED
                    motor_b.throttle = SPEED
                elif k == 's':
                    motor_a.throttle = -SPEED
                    motor_b.throttle = -SPEED
                elif k == 'a':
                    motor_a.throttle = SPEED * TURN_SCALE
                    motor_b.throttle = SPEED
                elif k == 'd':
                    motor_a.throttle = SPEED
                    motor_b.throttle = SPEED * TURN_SCALE
                elif k == ' ':
                    motor_a.throttle = 0
                    motor_b.throttle = 0
                elif k == 'q':
                    break

            if now - last_input > IDLE_STOP_SEC:
                motor_a.throttle = 0
                motor_b.throttle = 0

    finally:
        motor_a.throttle = 0
        motor_b.throttle = 0
        pca.deinit()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

if __name__ == "__main__":
    main()
