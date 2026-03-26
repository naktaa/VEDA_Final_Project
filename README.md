# Tank Merge Runtime

현재 브랜치는 `test/Tank/merge` 기준의 통합 런타임입니다.

구성:
- Qt(MQTT) / 컨트롤러(evdev) / auto goal tracking 통합 주행 제어
- 주행 우선순위: `Qt = controller > auto`
- `libcamera` 직접 캡처
- IMU 기반 gyro 보조 + LK 기반 translation 중심의 hybrid EIS
- `reference_tae.cpp` 방식 LK-only 보정 실행파일
- RTSP 송출
- `build` 내부 로컬 `ini` 기반 튜닝

상세 설계와 알고리즘 설명은 [docs/HYBRID_EIS_FRAMEWORK.md](/mnt/e/VEDA/VEDA_Final_Project/docs/HYBRID_EIS_FRAMEWORK.md)에 정리되어 있습니다.

## 의존성
- `cmake`
- `pkg-config`
- `libgpiod-dev`
- `wiringPi`
- `libmosquitto-dev`
- `libgstreamer1.0-dev`
- `libgstreamer-plugins-base1.0-dev`
- `libgstrtspserver-1.0-dev`
- `libcamera-dev`
- `OpenCV`

## 빌드
예전처럼 직접 빌드해서 `build` 안의 실행파일을 실행하면 됩니다.

```bash
mkdir -p build
cd build
cmake ..
make -j
```

생성되는 실행파일:
- `build/main`
- `build/lkonly`
- `build/main_raw`
- `build/calib`

## 실행 흐름
처음에는 calibration으로 로컬 설정 파일을 준비합니다.

```bash
cd build
sudo ./calib
```

`config_local.ini`가 없으면 템플릿 기준으로 생성하고 종료합니다.  
그 뒤 `config_local.ini`를 확인하고 다시 `sudo ./calib`를 실행하면 calibration 결과가 같은 파일의 `[calib]` 섹션에 저장됩니다.

최종 런타임은 아래처럼 실행합니다.

```bash
cd build
sudo ./main
```

`reference_tae.cpp` 방식의 LK + Kalman 보정만 따로 확인하려면 아래처럼 실행합니다.

```bash
cd build
sudo ./lkonly
```

보정 없는 원본 영상을 같은 RTSP 경로(`/cam`)로 확인하려면 아래 실행파일을 사용합니다.

```bash
cd build
sudo ./main_raw
```

## 주요 RTSP 경로
- active stream: `rtsp://<PI_IP>:8555/cam`

기본 포트/경로는 `config_local.ini`의 `[rtsp]` 섹션에서 바꿀 수 있습니다.

## 제어 모드
- Qt: `mqtt.control_topic`으로 들어오는 `tank_control` drive 명령
- controller: `/dev/input/event*` 기반 evdev 입력
- auto: `mqtt.goal_topic`, `mqtt.pose_topic`, `mqtt.safety_topic` 기반 goal tracking

Qt와 controller가 동시에 들어오면 최신 입력이 우선하고, 둘 다 없을 때만 auto가 모터 출력을 잡습니다.

## 설정 파일
Git에 올라가는 파일:
- `config_template.ini`

Git에 올라가지 않는 로컬 파일:
- `config_local.ini`

주요 섹션:
- `[camera]`
- `[imu]`
- `[eis]`
- `[rtsp]`
- `[mqtt]`
- `[auto]`
- `[motor]`
- `[manual]`
- `[controller]`
- `[calib]`

런타임 CLI 옵션은 의도적으로 없앴습니다. 파라미터 조정은 `config_local.ini` 기준으로 합니다.

## 권장 순서
1. `build/calib`로 bias와 `imu_offset_ms`를 먼저 저장합니다.
2. `config_local.ini`에서 `exposure_us`, `crop_budget_percent`, gyro/LK gain을 조정합니다.
3. `build/main`으로 EIS 화면을 확인합니다.
4. 필요할 때만 `build/main_raw`로 원본 화면을 확인합니다.
