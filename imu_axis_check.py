#!/usr/bin/env python3
import argparse
import os
import sys
import time
from collections import deque

import smbus2

# MPU-6050 registers
MPU6050_ADDR = 0x68
REG_PWR_MGMT_1 = 0x6B
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_WHO_AM_I = 0x75
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_XOUT_H = 0x43


def read_i16(bus, reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    val = (high << 8) | low
    if val > 32767:
        val -= 65536
    return val


def setup_mpu(bus):
    whoami = bus.read_byte_data(MPU6050_ADDR, REG_WHO_AM_I)
    if whoami != 0x68:
        print(f"[WARN] WHO_AM_I=0x{whoami:02X} (expected 0x68)")
    else:
        print(f"[OK] WHO_AM_I=0x{whoami:02X}")

    # Wake up device
    bus.write_byte_data(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00)
    # Sample rate divider: 1kHz / (1 + div)
    bus.write_byte_data(MPU6050_ADDR, REG_SMPLRT_DIV, 0x07)  # ~125Hz
    # DLPF config
    bus.write_byte_data(MPU6050_ADDR, REG_CONFIG, 0x03)
    # Gyro full scale = +/- 250 dps
    bus.write_byte_data(MPU6050_ADDR, REG_GYRO_CONFIG, 0x00)
    # Accel full scale = +/- 2g
    bus.write_byte_data(MPU6050_ADDR, REG_ACCEL_CONFIG, 0x00)


def read_accel_gyro(bus):
    ax = read_i16(bus, REG_ACCEL_XOUT_H)
    ay = read_i16(bus, REG_ACCEL_XOUT_H + 2)
    az = read_i16(bus, REG_ACCEL_XOUT_H + 4)

    gx = read_i16(bus, REG_GYRO_XOUT_H)
    gy = read_i16(bus, REG_GYRO_XOUT_H + 2)
    gz = read_i16(bus, REG_GYRO_XOUT_H + 4)
    return ax, ay, az, gx, gy, gz


def plot_mode(bus, rate_hz, bias_gx, bias_gy, bias_gz, window):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except Exception as e:
        print(f"[ERR] matplotlib import failed: {e}")
        print("Install with: sudo apt install python3-matplotlib")
        return

    accel_sens = 16384.0  # LSB/g
    gyro_sens = 131.0     # LSB/(deg/s)

    gx_buf = deque(maxlen=window)
    gy_buf = deque(maxlen=window)
    gz_buf = deque(maxlen=window)

    fig, ax = plt.subplots()
    line_gx, = ax.plot([], [], label="gx [deg/s]")
    line_gy, = ax.plot([], [], label="gy [deg/s]")
    line_gz, = ax.plot([], [], label="gz [deg/s]")

    ax.set_xlim(0, window)
    ax.set_ylim(-200, 200)
    ax.set_xlabel("sample index")
    ax.set_ylabel("gyro [deg/s]")
    ax.set_title("MPU6050 Gyro (Real-time)")
    ax.grid(True)
    ax.legend()
    text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    interval_ms = int(1000.0 / max(1.0, rate_hz))

    def update(_):
        ax_raw, ay_raw, az_raw, gx, gy, gz = read_accel_gyro(bus)
        gx_dps = (gx - bias_gx) / gyro_sens
        gy_dps = (gy - bias_gy) / gyro_sens
        gz_dps = (gz - bias_gz) / gyro_sens

        gx_buf.append(gx_dps)
        gy_buf.append(gy_dps)
        gz_buf.append(gz_dps)

        x = range(len(gx_buf))
        line_gx.set_data(x, list(gx_buf))
        line_gy.set_data(x, list(gy_buf))
        line_gz.set_data(x, list(gz_buf))

        abs_g = {"X": abs(gx_dps), "Y": abs(gy_dps), "Z": abs(gz_dps)}
        dominant = max(abs_g, key=abs_g.get)
        text.set_text(f"Dominant axis: {dominant}")

        return line_gx, line_gy, line_gz, text

    FuncAnimation(fig, update, interval=interval_ms, blit=False)
    print("Plot mode started. Close the window to stop.")
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="MPU6050 axis check (gyro/accel)")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    ap.add_argument("--rate", type=float, default=50.0, help="print rate in Hz (default: 50)")
    ap.add_argument("--calib", type=int, default=200, help="gyro bias samples at rest (default: 200)")
    ap.add_argument("--clear", action="store_true", help="clear screen each update (text mode)")
    ap.add_argument("--plot", action="store_true", help="plot gyro in realtime (requires matplotlib)")
    ap.add_argument("--window", type=int, default=200, help="plot history length (default: 200)")
    args = ap.parse_args()

    bus = smbus2.SMBus(args.bus)
    setup_mpu(bus)

    print("\nHold still for gyro bias calibration...")
    bias_gx = bias_gy = bias_gz = 0.0
    valid = 0
    for _ in range(args.calib):
        _, _, _, gx, gy, gz = read_accel_gyro(bus)
        bias_gx += gx
        bias_gy += gy
        bias_gz += gz
        valid += 1
        time.sleep(0.005)
    if valid:
        bias_gx /= valid
        bias_gy /= valid
        bias_gz /= valid
    print(f"Gyro bias: gx={bias_gx:.1f} gy={bias_gy:.1f} gz={bias_gz:.1f}")

    if args.plot:
        plot_mode(bus, args.rate, bias_gx, bias_gy, bias_gz, args.window)
        return

    # Sensitivity for default full scale
    accel_sens = 16384.0  # LSB/g
    gyro_sens = 131.0     # LSB/(deg/s)

    print("\nMove the camera slowly:")
    print("- Roll (tilt left/right): watch which gyro axis spikes.")
    print("- Pitch (tilt up/down): watch which gyro axis spikes.")
    print("- If sign is opposite, use sign=-1 in mapping.")
    print("")

    period = 1.0 / max(1.0, args.rate)
    use_clear = sys.stdout.isatty() and args.clear

    while True:
        ax, ay, az, gx, gy, gz = read_accel_gyro(bus)

        ax_g = ax / accel_sens
        ay_g = ay / accel_sens
        az_g = az / accel_sens

        gx_dps = (gx - bias_gx) / gyro_sens
        gy_dps = (gy - bias_gy) / gyro_sens
        gz_dps = (gz - bias_gz) / gyro_sens

        abs_g = {
            "x": abs(gx_dps),
            "y": abs(gy_dps),
            "z": abs(gz_dps),
        }
        dominant = max(abs_g, key=abs_g.get)

        if use_clear:
            os.system("clear")
        print("Gyro [deg/s]:  gx={:+7.2f}  gy={:+7.2f}  gz={:+7.2f}".format(gx_dps, gy_dps, gz_dps))
        print("Accel [g]  :  ax={:+6.3f}  ay={:+6.3f}  az={:+6.3f}".format(ax_g, ay_g, az_g))
        print(f"Dominant gyro axis: {dominant.upper()}")
        print("Tip: Roll the camera; whichever axis spikes = roll axis.")
        print("     Pitch the camera; whichever axis spikes = pitch axis.")

        time.sleep(period)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
