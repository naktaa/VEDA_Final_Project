#!/usr/bin/env python3

import time
import collections
import smbus2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ===== MPU6050 기본 설정 =====
MPU6050_ADDR = 0x68

REG_PWR_MGMT_1 = 0x6B
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_ACCEL_CONFIG = 0x1C
REG_WHO_AM_I = 0x75
REG_ACCEL_XOUT_H = 0x3B

bus = smbus2.SMBus(1)


def read_byte(reg):
    return bus.read_byte_data(MPU6050_ADDR, reg)


def write_byte(reg, value):
    bus.write_byte_data(MPU6050_ADDR, reg, value)


def read_word_2c(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    val = (high << 8) | low
    if val > 32767:
        val -= 65536
    return val


# ===== 초기 설정 =====
whoami = read_byte(REG_WHO_AM_I)
print(f"WHO_AM_I = 0x{whoami:02X}")

if whoami != 0x68:
    print("경고: WHO_AM_I 값 이상")

write_byte(REG_PWR_MGMT_1, 0x00)
write_byte(REG_SMPLRT_DIV, 0x07)
write_byte(REG_CONFIG, 0x06)
write_byte(REG_ACCEL_CONFIG, 0x00)

# ===== 그래프 버퍼 =====
WINDOW_SIZE = 200

ax_buf = collections.deque(maxlen=WINDOW_SIZE)
ay_buf = collections.deque(maxlen=WINDOW_SIZE)
az_buf = collections.deque(maxlen=WINDOW_SIZE)

# ===== matplotlib =====
fig, ax = plt.subplots()

line_ax, = ax.plot([], [], label="ax [g]")
line_ay, = ax.plot([], [], label="ay [g]")
line_az, = ax.plot([], [], label="az [g]")

ax.set_ylim(-2.0, 2.0)
ax.set_xlim(0, WINDOW_SIZE)
ax.set_xlabel("sample index")
ax.set_ylabel("acceleration [g]")
ax.set_title("MPU6050 Accelerometer (Real-time)")
ax.grid(True)
ax.legend()

# ===== 업데이트 =====
def update(frame):
    ax_raw = read_word_2c(REG_ACCEL_XOUT_H)
    ay_raw = read_word_2c(REG_ACCEL_XOUT_H + 2)
    az_raw = read_word_2c(REG_ACCEL_XOUT_H + 4)

    ax_g = ax_raw / 16384.0
    ay_g = ay_raw / 16384.0
    az_g = az_raw / 16384.0

    ax_buf.append(ax_g)
    ay_buf.append(ay_g)
    az_buf.append(az_g)

    x = range(len(ax_buf))

    line_ax.set_data(x, list(ax_buf))
    line_ay.set_data(x, list(ay_buf))
    line_az.set_data(x, list(az_buf))

    ax.set_xlim(0, max(len(ax_buf), WINDOW_SIZE))

    return line_ax, line_ay, line_az


ani = FuncAnimation(fig, update, interval=50)

print("그래프 시작")
plt.show()