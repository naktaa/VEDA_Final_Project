# Server Module

중앙 서버 모듈입니다. CCTV RTSP를 읽어 pose와 map/homography MQTT를 발행합니다.

## 구조

- `include/`, `src/`
- `config/`
  - `camera.yaml`
  - `H_img2world.yaml`
  - `server.yaml.example`
- `tools/`
  - `calib_camera.cpp`
  - `calib_aruco_homography.cpp`

## 빌드

루트 빌드:

```bash
cmake -S . -B build
cmake --build build --target server_main calib_camera calib_aruco_homography -j
```

출력:

- `build/server/main`
- `build/server/calib_camera`
- `build/server/calib_aruco_homography`

## 설정

런타임은 실행 파일 기준 `config/` 디렉터리를 읽습니다.

- `build/server/config/server.yaml`
- `build/server/config/camera.yaml`
- `build/server/config/H_img2world.yaml`

`server.yaml`이 없으면 `server.yaml.example` 또는 코드 기본값을 사용합니다.

## 실행

```bash
./build/server/calib_camera <체스보드_이미지_폴더> 8 6 3.0 ./build/server/config/camera.yaml
./build/server/calib_aruco_homography ./build/server/config/H_img2world.yaml
./build/server/main
```

## MQTT

- `wiserisk/p1/pose`
- `wiserisk/map/H_img2world`
- `wiserisk/map/graph`
