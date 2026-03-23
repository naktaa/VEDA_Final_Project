# Tank MQTT Drive + RTSP + tiltVR

This project runs a Raspberry Pi based tank controller with four subsystems in one process:

- MQTT drive control for the tank motors
- RTSP video streaming
- HTTP/MJPEG VR web viewer
- Phone IMU driven pan/tilt using two SG90 servos on a PCA9685 board

The PTZ path no longer uses the old serial servo packet protocol. It now drives two SG90 servos through `I2C -> PCA9685 -> SG90`.

## Runtime flow

1. `src/mqtt_tank_drive.cpp` starts motor control, camera capture, RTSP, HTTP VR, MQTT, and PTZ worker threads.
2. The phone opens `http://<PI_IP>:8000/web/`.
3. `web/tilt_vr/app.js` reads `deviceorientation` data from the phone and posts it to `POST /imu`.
4. `src/http_vr_server.cpp` forwards that IMU payload to `PtzController`.
5. `src/ptz_control.cpp` smooths the sensor values, maps them to pan/tilt angles, and writes PWM output through PCA9685.
6. If phone IMU input stops for `imu_timeout_ms`, MQTT PTZ commands become the fallback input.

## Build

```bash
mkdir -p build
cd build
cmake ..
make -j
```

Required packages:

- `wiringPi`
- `libmosquitto-dev`
- `libgstreamer1.0-dev`
- `libgstrtspserver-1.0-dev`
- `cpp-httplib` headers (`httplib.h`)
- `nlohmann-json3-dev`
- `libopencv-dev`
- GStreamer plugins for `libcamerasrc` and `v4l2h264enc`

## Run

```bash
./main
```

The build also copies the binary as `./mqtt_tank_drive`.

## Default endpoints

- Web UI: `http://<PI_IP>:8000/web/`
- MJPEG stream: `http://<PI_IP>:8000/stream.mjpg`
- Health check: `http://<PI_IP>:8000/health`
- Latest IMU/PTZ state: `http://<PI_IP>:8000/imu/latest`
- RTSP stream: `rtsp://<PI_IP>:8555/cam`

## PTZ hardware defaults

- I2C device: `/dev/i2c-1`
- PCA9685 address: `0x40`
- PWM frequency: `50Hz`
- Pan channel: `0`
- Tilt channel: `1`
- Pan center/left/right: `90 / 180 / 0`
- Tilt center/up/down: `90 / 0 / 180`

## Options

```bash
--host <addr>
--port <n>
--topic <topic>
--no-rtsp
--rtsp-port <port>
--rtsp-path <path>
--rtsp-launch <launch>
--no-http-vr
--http-port <port>
--i2c-dev <path>
--i2c-addr <n>
--pan-channel <n>
--tilt-channel <n>
```

## MQTT message format

Example payload:

```json
{
  "type": "tank_control",
  "target": "tank",
  "group": "ptz",
  "command": "pan_left",
  "active": true,
  "source": "qt_main_window",
  "ts_ms": 1773890000000
}
```

Accepted PTZ commands:

- `pan_left`
- `pan_right`
- `tilt_up`
- `tilt_down`

Accepted drive commands:

- `forward`
- `backward`
- `turn_left`
- `turn_right`

## HTTP IMU payload

```json
{
  "type": "imu",
  "t": 1773890000000,
  "pitch": -12.4,
  "roll": 8.1,
  "yaw": 92.0
}
```

The web client zero-calibrates on connect, then continuously sends recentered phone IMU values to `/imu`.
