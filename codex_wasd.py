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
FREQ_HZ = 200  # adjust in 100~1600 if needed

# Yahboom Jetbot Hardware Manual (PCA9685 mapping)
# Right motor: IN1A ch8, IN1B ch9
# Left motor : IN2A ch10, IN2B ch11
RIGHT_IN1 = 8
RIGHT_IN2 = 9
LEFT_IN1 = 10
LEFT_IN2 = 11

SPEED = 1.0
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

    motor_right = motor.DCMotor(pca.channels[RIGHT_IN1], pca.channels[RIGHT_IN2])
    motor_left = motor.DCMotor(pca.channels[LEFT_IN1], pca.channels[LEFT_IN2])

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    last_input = time.monotonic()
    try:
        print("WASD control start (q: quit). Stop when idle.")
        while True:
            key = read_key(0.05)
            now = time.monotonic()

            if key:
                last_input = now
                k = key.lower()

                if k == 'w':
                    motor_left.throttle = SPEED
                    motor_right.throttle = SPEED
                elif k == 's':
                    motor_left.throttle = -SPEED
                    motor_right.throttle = -SPEED
                elif k == 'a':
                    motor_left.throttle = SPEED * TURN_SCALE
                    motor_right.throttle = SPEED
                elif k == 'd':
                    motor_left.throttle = SPEED
                    motor_right.throttle = SPEED * TURN_SCALE
                elif k == ' ':
                    motor_left.throttle = 0
                    motor_right.throttle = 0
                elif k == 'q':
                    break

            if now - last_input > IDLE_STOP_SEC:
                motor_left.throttle = 0
                motor_right.throttle = 0

    finally:
        motor_left.throttle = 0
        motor_right.throttle = 0
        pca.deinit()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()
