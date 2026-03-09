# tilting_VR (C++ MVP)

This project streams Raspberry Pi camera video to a phone in VR split-view and
uses the phone orientation sensor to control pan/tilt servos over serial
(`/dev/serial0`).

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
  libopencv-dev \
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

Enable UART serial first:

```bash
sudo raspi-config
# Interface Options -> Serial Port
# - Login shell over serial? -> No
# - Enable serial port hardware? -> Yes
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

## Serial servo mapping (calibrated)

- Device: `/dev/serial0` @ `115200`
- Servo IDs: `pan=1`, `tilt=2`
- Center: `pan=1800`, `tilt=2400`
- Left 45 deg: `pan=2650`, Right 45 deg: `pan=1150`
- Up 45 deg: `tilt=2900`, Down 45 deg: `tilt=2200` (mechanical limit)
- IMU clamp: +/- 45 deg (piecewise linear mapping)

Update constants in `server/src/main.cpp` if your calibration differs.
