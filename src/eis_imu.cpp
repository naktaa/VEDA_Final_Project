#include "eis_imu.hpp"

#include <cstdio>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <linux/i2c-dev.h>

#include "eis_globals.hpp"

static double pick_axis(int axis, double x, double y, double z) {
    switch (axis) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: return z;
    }
}

static int16_t to_i16(uint8_t h, uint8_t l) {
    return (int16_t)((h << 8) | l);
}

static bool i2c_write_byte(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (write(fd, buf, 2) == 2);
}

static bool i2c_read_bytes(int fd, uint8_t reg, uint8_t* data, int len) {
    if (write(fd, &reg, 1) != 1) return false;
    return (read(fd, data, len) == len);
}

void imu_loop() {
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        perror("[IMU] open");
        return;
    }
    if (ioctl(fd, I2C_SLAVE, IMU_ADDR) < 0) {
        perror("[IMU] ioctl");
        close(fd);
        return;
    }

    i2c_write_byte(fd, 0x6B, 0x00); // wake
    i2c_write_byte(fd, 0x1A, 0x03); // DLPF cfg
    i2c_write_byte(fd, 0x19, 0x01); // SMPLRT_DIV=1 => 1kHz/(1+1)=500Hz
    i2c_write_byte(fd, 0x1B, 0x00); // gyro full scale +/-250dps
    usleep(100000);

    fprintf(stderr, "[IMU] Calibrating gyro (%d samples)...\n", CALIB_SAMPLES);
    double bias_gx = 0, bias_gy = 0, bias_gz = 0;
    int valid = 0;
    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        uint8_t buf[6];
        if (!i2c_read_bytes(fd, 0x43, buf, 6)) continue;
        bias_gx += to_i16(buf[0], buf[1]);
        bias_gy += to_i16(buf[2], buf[3]);
        bias_gz += to_i16(buf[4], buf[5]);
        valid++;
        usleep(2000);
    }
    if (valid > 0) {
        bias_gx /= valid;
        bias_gy /= valid;
        bias_gz /= valid;
    }
    fprintf(stderr, "[IMU] Calibration done (bias: gx=%.1f gy=%.1f gz=%.1f)\n", bias_gx, bias_gy, bias_gz);

    g_imu_ready = true;
    fprintf(stderr, "[IMU] Ready (target: %dHz, buffer: %d samples)\n", IMU_TARGET_HZ, IMU_STORE_SIZE);

    int64_t last_time_ns = clock_ns(CLOCK_MONOTONIC_RAW);

    while (g_running) {
        int64_t loop_start_ns = clock_ns(CLOCK_MONOTONIC_RAW);
        uint8_t buf[6];
        if (!i2c_read_bytes(fd, 0x43, buf, 6)) {
            usleep(500);
            continue;
        }
        int64_t now_ns = clock_ns(CLOCK_MONOTONIC_RAW);
        double dt = (now_ns - last_time_ns) / 1e9;
        last_time_ns = now_ns;
        if (dt <= 0 || dt > 0.1) {
            usleep(500);
            continue;
        }

        double raw_gx = (double)to_i16(buf[0], buf[1]);
        double raw_gy = (double)to_i16(buf[2], buf[3]);
        double raw_gz = (double)to_i16(buf[4], buf[5]);
        double gx_rad = ((raw_gx - bias_gx) / GYRO_SENSITIVITY) * (CV_PI / 180.0);
        double gy_rad = ((raw_gy - bias_gy) / GYRO_SENSITIVITY) * (CV_PI / 180.0);
        double gz_rad = ((raw_gz - bias_gz) / GYRO_SENSITIVITY) * (CV_PI / 180.0);

        double roll_rate = IMU_SIGN_ROLL * pick_axis(IMU_AXIS_ROLL, gx_rad, gy_rad, gz_rad);
        double pitch_rate = IMU_SIGN_PITCH * pick_axis(IMU_AXIS_PITCH, gx_rad, gy_rad, gz_rad);
        double yaw_rate = IMU_SIGN_YAW * pick_axis(IMU_AXIS_YAW, gx_rad, gy_rad, gz_rad);
        double ts = now_ms();
        GyroSample sample;
        sample.t_ms = ts;
        sample.w_rad = cv::Vec3d(roll_rate, pitch_rate, yaw_rate);
        sample.raw = cv::Vec3d(raw_gx, raw_gy, raw_gz);
        g_gyro_buffer.push(sample);
        g_last_gyro_sample = sample;

        if (dt > 0) {
            double hz = 1.0 / dt;
            double prev = g_imu_actual_hz.load();
            g_imu_actual_hz = (prev <= 0) ? hz : (prev * 0.9 + hz * 0.1);
        }
        int64_t elapsed_ns = clock_ns(CLOCK_MONOTONIC_RAW) - loop_start_ns;
        int64_t sleep_ns = IMU_TARGET_PERIOD_NS - elapsed_ns;
        if (sleep_ns > 0) {
            struct timespec ts_sleep;
            ts_sleep.tv_sec = sleep_ns / 1000000000LL;
            ts_sleep.tv_nsec = sleep_ns % 1000000000LL;
            nanosleep(&ts_sleep, nullptr);
        }
    }
    close(fd);
    fprintf(stderr, "[IMU] Thread exiting\n");
}

