"""
WASD 팬틸트 테스트 (Python 버전)
기존 pantilt_test.cpp와 동일한 동작
"""
import serial
import sys
import tty
import termios
import select
import time

DEV = "/dev/serial0"
BAUD = 115200
STEP = 50

PAN_MIN, PAN_MAX = 1000, 3200
TILT_MIN, TILT_MAX = 1600, 3400

pan = 1800
tilt = 2400

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def make_write_pos(sid, pos):
    pos_H = (pos >> 8) & 0xFF
    pos_L = pos & 0xFF
    time_H = 0x00
    time_L = 0x0A
    length = 0x07
    cmd = 0x03
    addr = 0x2A
    chk = (~(sid + length + cmd + addr + pos_H + pos_L + time_H + time_L)) & 0xFF
    return bytes([0xFF, 0xFF, sid, length, cmd, addr, pos_H, pos_L, time_H, time_L, chk])

def get_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0.1)
    if dr:
        return sys.stdin.read(1)
    return None

ser = serial.Serial(DEV, BAUD, timeout=0.1)

# 초기 위치
ser.write(make_write_pos(1, pan))
ser.flush()
time.sleep(0.05)
ser.write(make_write_pos(2, tilt))
ser.flush()

print("=== PAN/TILT WASD TEST (Python) ===")
print("W/S : tilt up/down")
print("A/D : pan left/right")
print("SPACE: center")
print("P: print current value")
print("Q: quit")
print(f"Current -> PAN={pan} TILT={tilt}")

old = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())
    while True:
        key = get_key()
        if key is None:
            continue
        moved = False
        key = key.lower()
        if key == 'a':
            pan = clamp(pan + STEP, PAN_MIN, PAN_MAX)
            moved = True
        elif key == 'd':
            pan = clamp(pan - STEP, PAN_MIN, PAN_MAX)
            moved = True
        elif key == 'w':
            tilt = clamp(tilt + STEP, TILT_MIN, TILT_MAX)
            moved = True
        elif key == 's':
            tilt = clamp(tilt - STEP, TILT_MIN, TILT_MAX)
            moved = True
        elif key == ' ':
            pan = 1800
            tilt = 2400
            moved = True
        elif key == 'p':
            print(f"\nCurrent -> PAN={pan} TILT={tilt}")
        elif key == 'q':
            print("\nQuit")
            break

        if moved:
            ser.write(make_write_pos(1, pan))
            ser.flush()
            time.sleep(0.01)
            ser.write(make_write_pos(2, tilt))
            ser.flush()
            print(f"\rPAN={pan}  TILT={tilt}        ", end="", flush=True)
            time.sleep(0.03)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    ser.close()
