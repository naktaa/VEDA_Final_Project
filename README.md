# VEDA 서버 브랜치

이 브랜치는 CCTV 기반 위치 추정과 맵/호모그래피 배포만 담당하는 서버 전용 브랜치입니다.

## 구성

- `calib_file/calib_camera.cpp`
  - 체스보드 이미지로 `config/camera.yaml` 생성
- `calib_file/calib_aruco_homography.cpp`
  - 바닥 ArUco 4점으로 `config/H_img2world.yaml` 생성
- `server_main.cpp`
  - RTSP 입력을 읽어 `wiserisk/p1/pose` 발행
  - `config/H_img2world.yaml`을 주기적으로 읽어 `wiserisk/map/H_img2world`, `wiserisk/map/graph` 발행

## 중요한 점

- `camera.yaml`은 실제로 사용합니다.
  - ArUco pose 추정
  - yaw 계산
  - `R_world_cam` 추정
- `H_img2world.yaml`은 실제로 사용합니다.
  - 이미지 좌표를 월드 좌표 `x, y`로 변환
  - 맵/호모그래피 MQTT 발행

즉 두 yaml 모두 서버 주행 정확도에 직접 영향을 줍니다.

## 빌드

필수:

- OpenCV
- libmosquitto
- CMake 3.16 이상

```bash
cmake -S . -B build
cmake --build build --target calib_camera calib_aruco_homography server_main -j$(nproc)
```

## 실행 파일

### 1. 카메라 캘리브레이션

```bash
./build/calib_camera <체스보드_이미지_폴더> 8 6 3.0 ./config/camera.yaml
```

출력:

- `config/camera.yaml`

### 2. 바닥 호모그래피 캘리브레이션

```bash
./build/calib_aruco_homography ./config/H_img2world.yaml
```

출력:

- `config/H_img2world.yaml`

### 3. 서버 메인 실행

```bash
./build/server_main \
  "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp" \
  192.168.100.10 \
  1883 \
  wiserisk/p1/pose \
  ./config/H_img2world.yaml \
  ./config/camera.yaml \
  0.17 \
  0.17 \
  wiserisk/map/H_img2world \
  wiserisk/map/graph \
  1000
```

인자 순서:

1. `rtsp_url`
2. `mqtt_host`
3. `mqtt_port`
4. `pose_topic`
5. `homography_yaml`
6. `camera_yaml`
7. `marker_size`
8. `cube_size`
9. `homography_topic`
10. `map_topic`
11. `publish_interval_ms`

## MQTT 토픽

- 발행: `wiserisk/p1/pose`
- 발행: `wiserisk/map/H_img2world`
- 발행: `wiserisk/map/graph`

## 권장 운영 순서

1. 카메라 위치나 렌즈 상태가 바뀌었으면 `calib_camera` 실행
2. 카메라 위치나 바닥 마커 배치가 바뀌었으면 `calib_aruco_homography` 실행
3. `server_main` 실행

## 리팩토링 방향

기존에는 `p1_tracker`와 `homography_mqtt_pub`를 따로 실행했지만, 현재는 내부 모듈로 통합해서 `server_main` 하나로 운용합니다.

내부 역할은 다음처럼 분리되어 있습니다.

- `src/server_utils.cpp`
  - yaml 로드, RTSP 연결, MQTT publish, 공통 수학/직렬화 유틸
- `src/homography_publisher.cpp`
  - `H_img2world.yaml` 재로드 및 `wiserisk/map/*` 발행
- `src/pose_tracker.cpp`
  - ArUco 검출, `x/y/yaw` 계산, `wiserisk/p1/pose` 발행
- `src/server_app.cpp`
  - 전체 실행 순서와 루프 orchestration
