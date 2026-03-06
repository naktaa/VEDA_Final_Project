import time
import board
import busio
from adafruit_pca9685 import PCA9685

# PCA9685 address (YB-EJF01 manual)
I2C_ADDR = 0x40
FREQ_HZ = 200  # 낮게 설정하면 토크가 잘 나오는 경우가 많음

# Motor mapping (manual 기준)
RIGHT_IN1 = 8   # Right motor IN1A
RIGHT_IN2 = 9   # Right motor IN1B
LEFT_IN1  = 10  # Left motor IN2A
LEFT_IN2  = 11  # Left motor IN2B

# 방향이 반대면 True로 바꾸기
INVERT_RIGHT = False
INVERT_LEFT = False

KICK_SEC = 0.2
RUN_SEC = 1.5
DUTY_KICK = 0xFFFF
DUTY_RUN = 0xFFFF  # 토크 최대 테스트

def set_ch(pca, ch, duty):
    pca.channels[ch].duty_cycle = duty

def drive_pair(pca, in1, in2, forward=True):
    if forward:
        set_ch(pca, in1, DUTY_KICK); set_ch(pca, in2, 0); time.sleep(KICK_SEC)
        set_ch(pca, in1, DUTY_RUN);  set_ch(pca, in2, 0); time.sleep(RUN_SEC)
    else:
        set_ch(pca, in1, 0); set_ch(pca, in2, DUTY_KICK); time.sleep(KICK_SEC)
        set_ch(pca, in1, 0); set_ch(pca, in2, DUTY_RUN);  time.sleep(RUN_SEC)

def stop_pair(pca, in1, in2, pause=0.5):
    set_ch(pca, in1, 0); set_ch(pca, in2, 0); time.sleep(pause)

def maybe_invert(in1, in2, inv):
    return (in2, in1) if inv else (in1, in2)

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c, address=I2C_ADDR)
    pca.frequency = FREQ_HZ

    r1, r2 = maybe_invert(RIGHT_IN1, RIGHT_IN2, INVERT_RIGHT)
    l1, l2 = maybe_invert(LEFT_IN1, LEFT_IN2, INVERT_LEFT)

    try:
        print("FORWARD")
        drive_pair(pca, r1, r2, True)
        drive_pair(pca, l1, l2, True)
        stop_pair(pca, r1, r2); stop_pair(pca, l1, l2)

        print("BACKWARD")
        drive_pair(pca, r1, r2, False)
        drive_pair(pca, l1, l2, False)
        stop_pair(pca, r1, r2); stop_pair(pca, l1, l2)

        print("SPIN")
        drive_pair(pca, r1, r2, True)
        drive_pair(pca, l1, l2, False)
        stop_pair(pca, r1, r2); stop_pair(pca, l1, l2)

    finally:
        for ch in (RIGHT_IN1, RIGHT_IN2, LEFT_IN1, LEFT_IN2):
            pca.channels[ch].duty_cycle = 0
        pca.deinit()

if __name__ == "__main__":
    main()
