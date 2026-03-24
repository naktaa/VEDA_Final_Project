# VEDA 서버 브랜치

이 브랜치는 CCTV 기반 위치 추정과 맵/호모그래피 배포만 담당하는 서버 전용 브랜치입니다.

## 구성

- `calib_file/calib_camera.cpp`
  - 체스보드 이미지로 `config/camera.yaml` 생성
- `calib_file/calib_aruco_homography.cpp`
  - 바닥 ArUco 4점으로 `config/H_img2world.yaml` 생성
- `server_main.cpp`
  - 최종 실행 파일 이름은 `main`
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
mkdir -p build
cd build
cmake ..
make -j
```

## 실행 파일

### 1. 카메라 캘리브레이션

```bash
./calib_camera <체스보드_이미지_폴더> 8 6 3.0 ../config/camera.yaml
```

출력:

- `config/camera.yaml`

### 2. 바닥 호모그래피 캘리브레이션

```bash
./calib_aruco_homography ../config/H_img2world.yaml
```

출력:

- `config/H_img2world.yaml`

### 3. 서버 메인 실행

```bash
./main
```

동작 방식:

1. `build/main`은 자동으로 상위 디렉터리의 `config/camera.yaml`, `config/H_img2world.yaml`를 사용합니다.
2. RTSP URL, MQTT broker, 토픽, marker 크기는 코드 기본값을 사용합니다.
3. 즉 서버 메인은 실행 인자를 받지 않습니다.

## MQTT 토픽

- 발행: `wiserisk/p1/pose`
- 발행: `wiserisk/map/H_img2world`
- 발행: `wiserisk/map/graph`

## 고정 기본값

- RTSP URL: `rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp`
- MQTT broker: `192.168.100.10:1883`
- marker size: `0.17`
- cube size: `0.17`
- homography publish 주기: `1000ms`

## 권장 운영 순서

1. 카메라 위치나 렌즈 상태가 바뀌었으면 `calib_camera` 실행
2. 카메라 위치나 바닥 마커 배치가 바뀌었으면 `calib_aruco_homography` 실행
3. `main` 실행

## 리팩토링 방향

기존에는 `p1_tracker`와 `homography_mqtt_pub`를 따로 실행했지만, 현재는 내부 모듈로 통합해서 최종 실행 파일 `main` 하나로 운용합니다.

내부 역할은 다음처럼 분리되어 있습니다.

- `src/server_utils.cpp`
  - yaml 로드, RTSP 연결, MQTT publish, 공통 수학/직렬화 유틸
- `src/homography_publisher.cpp`
  - `H_img2world.yaml` 재로드 및 `wiserisk/map/*` 발행
- `src/pose_tracker.cpp`
  - ArUco 검출, `x/y/yaw` 계산, `wiserisk/p1/pose` 발행
- `src/server_app.cpp`
  - 전체 실행 순서와 루프 orchestration
