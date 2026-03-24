# VEDA Server Branch

This branch contains only the server-side vision and calibration flow split out from `test/server`.

## Responsibility

- `p1_tracker.cpp`
  - Reads the RTSP stream, detects ArUco markers, and publishes `wiserisk/p1/pose`.
  - Subscribes to `wiserisk/map/H_img2world` to refresh the homography at runtime.
- `calib_file/homography_mqtt_pub.cpp`
  - Publishes `wiserisk/map/H_img2world` and `wiserisk/map/graph`.
- `calib_file/calib_aruco_homography.cpp`
  - Builds `config/H_img2world.yaml` from floor markers.
- `calib_file/calib_camera.cpp`
  - Builds `config/camera.yaml` from chessboard images.
- `config/H_img2world.yaml`, `config/camera.yaml`
  - Sample server-side calibration/config inputs.

## Out Of Scope

- No RC motor control code
- No `wiserisk/rc/status` publisher
- No RC test publisher or RC-only runtime config

## MQTT Interface

- Publish: `wiserisk/p1/pose`
- Publish: `wiserisk/map/H_img2world`
- Publish: `wiserisk/map/graph`
- Subscribe: `wiserisk/map/H_img2world` (`p1_tracker`)

## Build

Prerequisites:

- OpenCV
- libmosquitto
- CMake 3.16+

```bash
cmake -S . -B build
cmake --build build --target p1_tracker homography_mqtt_pub calib_aruco_homography calib_camera -j$(nproc)
```

## Run

Publish homography and map:

```bash
./build/homography_mqtt_pub \
  ./config/H_img2world.yaml \
  192.168.100.10 \
  1883 \
  wiserisk/map/H_img2world \
  wiserisk/map/graph \
  1000 \
  1
```

Run tracker:

```bash
./build/p1_tracker \
  "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp" \
  192.168.100.10 \
  1883 \
  wiserisk/p1/pose \
  ./config/H_img2world.yaml \
  ./config/camera.yaml \
  0.17 \
  0.17
```

Create homography YAML:

```bash
./build/calib_aruco_homography ./config/H_img2world.yaml
```

Create camera YAML:

```bash
./build/calib_camera <image_dir> 8 6 3.0 ./config/camera.yaml
```
