# tilting_VR (C++ MVP)

This project streams Raspberry Pi camera video to a phone in VR split-view and
uses the phone orientation sensor to control pan/tilt servos (PCA9685 + MG90S).

## What changed

- Backend is now C++ (`server/src/main.cpp`)
- Video path: MJPEG stream (`/stream.mjpg`)
- IMU control path: HTTP POST (`/imu`)
- Web client still runs in browser (`web/`) and sends phone orientation values

## Gyro usage

Yes, the app uses the phone orientation sensor.

- In `web/app.js`, `deviceorientation` is read (`alpha`, `beta`, `gamma`)
- Values are recentered and sent every 33ms to `/imu`
- C++ server maps:
  - `pitch` -> tilt servo (up/down)
  - `roll` -> pan servo (left/right)

## Folder layout

- `server/CMakeLists.txt`
- `server/src/main.cpp`
- `web/index.html`
- `web/app.js`
- `web/style.css`

## Raspberry Pi dependencies

Install packages on Raspberry Pi OS (example):

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev i2c-tools \
  nlohmann-json3-dev
```

You also need `cpp-httplib` header (`httplib.h`) available in include path.

## Build

```bash
cd tilting_VR/server
mkdir -p build
cd build
cmake ..
cmake --build . -j
```

## Run

Enable I2C first:

```bash
sudo raspi-config
# Interface Options -> I2C -> Enable
sudo reboot
```

Run server:

```bash
cd tilting_VR/server/build
./tilting_vr_server
```

Open on phone (same Wi-Fi):

```text
http://<raspberry-pi-ip>:8000/web/
```

Tap `Connect + Start VR`, allow orientation permission, then mount the phone in
your VR headset.

## Default PCA9685 mapping

- I2C bus: `1`
- PCA9685 address: `0x40`
- pan servo channel: `0`
- tilt servo channel: `1`

Update constants in `server/src/main.cpp` if your wiring differs.
