#!/usr/bin/env python3

import time
import collections
import smbus2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ===== MPU6050 basic setup =====
MPU6050_ADDR = 0x68

REG_PWR_MGMT_1 = 0x6B
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_WHO_AM_I = 0x75
REG_GYRO_XOUT_H = 0x43

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


# ===== init =====
whoami = read_byte(REG_WHO_AM_I)
print(f"WHO_AM_I = 0x{whoami:02X}")
if whoami != 0x68:
    print("Warning: WHO_AM_I mismatch")

write_byte(REG_PWR_MGMT_1, 0x00)
# sample rate = 1kHz / (1 + div) when DLPF enabled
write_byte(REG_SMPLRT_DIV, 0x04)  # ~200Hz
write_byte(REG_CONFIG, 0x03)      # DLPF
write_byte(REG_GYRO_CONFIG, 0x00) # +/-250 dps

time.sleep(0.1)

# ===== bias calibration =====
CALIB_SAMPLES = 300
print(f"Calibrating gyro bias ({CALIB_SAMPLES} samples). Keep still...")
bx = by = bz = 0.0
for _ in range(CALIB_SAMPLES):
    gx = read_word_2c(REG_GYRO_XOUT_H)
    gy = read_word_2c(REG_GYRO_XOUT_H + 2)
    gz = read_word_2c(REG_GYRO_XOUT_H + 4)
    bx += gx
    by += gy
    bz += gz
    time.sleep(0.002)
bx /= CALIB_SAMPLES
by /= CALIB_SAMPLES
bz /= CALIB_SAMPLES
print(f"Bias (raw): gx={bx:.1f} gy={by:.1f} gz={bz:.1f}")

GYRO_SENS = 131.0  # LSB/(deg/s) for +/-250 dps

# ===== plot buffers =====
WINDOW_SIZE = 200

gx_buf = collections.deque(maxlen=WINDOW_SIZE)
gy_buf = collections.deque(maxlen=WINDOW_SIZE)
gz_buf = collections.deque(maxlen=WINDOW_SIZE)

angx_buf = collections.deque(maxlen=WINDOW_SIZE)
angy_buf = collections.deque(maxlen=WINDOW_SIZE)
angz_buf = collections.deque(maxlen=WINDOW_SIZE)

angx = angy = angz = 0.0
last_t = time.monotonic()

# ===== matplotlib =====
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

line_gx, = ax1.plot([], [], label="gx (deg/s)")
line_gy, = ax1.plot([], [], label="gy (deg/s)")
line_gz, = ax1.plot([], [], label="gz (deg/s)")

line_ax, = ax2.plot([], [], label="angle x (deg)")
line_ay, = ax2.plot([], [], label="angle y (deg)")
line_az, = ax2.plot([], [], label="angle z (deg)")

RATE_YLIM = 80   # 좁게 보기 (deg/s)
ANG_YLIM = 30    # 좁게 보기 (deg)

ax1.set_ylim(-RATE_YLIM, RATE_YLIM)
ax1.set_ylabel("gyro rate (deg/s)")
ax1.grid(True)
ax1.legend()

ax2.set_ylim(-ANG_YLIM, ANG_YLIM)
ax2.set_xlabel("sample index")
ax2.set_ylabel("integrated angle (deg)")
ax2.grid(True)
ax2.legend()


def update(frame):
    global angx, angy, angz, last_t

    now = time.monotonic()
    dt = now - last_t
    last_t = now
    if dt <= 0.0 or dt > 0.1:
        dt = 0.005

    gx_raw = read_word_2c(REG_GYRO_XOUT_H)
    gy_raw = read_word_2c(REG_GYRO_XOUT_H + 2)
    gz_raw = read_word_2c(REG_GYRO_XOUT_H + 4)

    gx = (gx_raw - bx) / GYRO_SENS
    gy = (gy_raw - by) / GYRO_SENS
    gz = (gz_raw - bz) / GYRO_SENS

    angx += gx * dt
    angy += gy * dt
    angz += gz * dt

    gx_buf.append(gx)
    gy_buf.append(gy)
    gz_buf.append(gz)
    angx_buf.append(angx)
    angy_buf.append(angy)
    angz_buf.append(angz)

    x = range(len(gx_buf))
    line_gx.set_data(x, list(gx_buf))
    line_gy.set_data(x, list(gy_buf))
    line_gz.set_data(x, list(gz_buf))

    line_ax.set_data(x, list(angx_buf))
    line_ay.set_data(x, list(angy_buf))
    line_az.set_data(x, list(angz_buf))

    ax1.set_xlim(0, max(len(gx_buf), WINDOW_SIZE))
    ax2.set_xlim(0, max(len(gx_buf), WINDOW_SIZE))

    # dominant axis indicator
    axis = "x"
    vmax = abs(gx)
    if abs(gy) > vmax:
        axis = "y"
        vmax = abs(gy)
    if abs(gz) > vmax:
        axis = "z"
    ax1.set_title(f"Gyro rates (dominant axis: {axis})")

    return line_gx, line_gy, line_gz, line_ax, line_ay, line_az


ani = FuncAnimation(fig, update, interval=50)
print("Start gyro axis plot. Rotate around yaw and watch which axis dominates.")
plt.show()
