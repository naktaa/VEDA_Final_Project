# Hybrid EIS Framework

## 개요
이 구현은 `LK-only` 구조를 그대로 확장하지 않고, 아래 모듈로 나눈 뒤 `main`과 `calib_offset`이 같은 코어를 공유하도록 재구성한 버전입니다.

- `config`
- `timebase`
- `capture`
- `imu`
- `lk`
- `gyro`
- `fusion`
- `rtsp`
- `drive`

최종 방향은 `LK main + gyro assist + turn-follow state machine`입니다.

## 전체 프레임워크
### 공통 모듈
- `include/app_config.hpp`, `src/app_config.cpp`
  로컬 `ini` 로딩/저장
- `include/timebase.hpp`, `src/timebase.cpp`
  `CLOCK_MONOTONIC_RAW` 기준 시간축
- `include/libcamera_capture.hpp`, `src/libcamera_capture.cpp`
  `libcamera` 직접 캡처와 `SensorTimestamp + ExposureTime/2`
- `include/imu_reader.hpp`, `src/imu_reader.cpp`
  MPU-6050 읽기, bias 적용, FIFO/interrupt 우선, polling fallback
- `include/lk_tracker.hpp`, `src/lk_tracker.cpp`
  LK optical flow + RANSAC affine 기반 visual motion 추정
- `include/gyro_eis.hpp`, `src/gyro_eis.cpp`
  gyro buffer, quaternion 적분, 회전 homography 계산
- `include/hybrid_eis.hpp`, `src/hybrid_eis.cpp`
  LK + gyro fusion, state machine, crop clamp
- `include/rtsp_server.hpp`, `src/rtsp_server.cpp`
  `/cam`과 `/raw` RTSP 송출
- `src/mqtt_drive.cpp`, `src/tank_drive.cpp`
  기존 탱크 주행 제어 재사용

### 실행 흐름
- `src/main.cpp`
  최종 통합 런타임
- `src/calib_offset.cpp`
  bias/offset calibration 전용 실행 흐름

## 실행파일
### `main`
- 목적: 실제 주행용 통합 런타임
- 실행: `build/main`
- 내부 동작:
  - `config_local.ini` 로드
  - MQTT 연결
  - libcamera 캡처 시작
  - IMU 읽기 시작
  - hybrid EIS 수행
  - `/cam`, `/raw` RTSP 송출

### `calib`
- 목적: bias 측정, IMU-camera offset 추정, yaw 축/sign 검토
- 실행: `build/calib`
- 내부 동작:
  - 정지 상태 bias 측정
  - 약 8초간 LK rotation 수집
  - gyro 적분과 LK rotation correlation 기반 offset sweep
  - `[calib]` 섹션 업데이트

## 사용된 핵심 로직
## 1. 카메라 타임스탬프
프레임 시각은 다음 기준으로 계산합니다.

```text
frame_time = SensorTimestamp + ExposureTime / 2
```

이 값은 `CLOCK_MONOTONIC_RAW` 기준 ms로 변환되어 IMU 시간축과 맞춰집니다.

이 구조를 쓴 이유:
- `push` 시점 timestamp 대신 센서 기준 timestamp를 사용하기 위해
- 나중에 gyro integration에 직접 사용할 수 있도록 하기 위해
- 노출 midpoint 기준으로 맞추는 쪽이 고주파 진동에서 더 안정적이기 때문

## 2. IMU 읽기 구조
우선순위는 아래와 같습니다.

1. `FIFO + interrupt`
2. `FIFO polling`
3. `direct polling`

`imu.int_pin_wpi`가 설정되면 `waitForInterrupt()` 기반으로 data-ready를 기다리고 FIFO를 비웁니다.  
핀 설정이 없으면 FIFO polling 또는 일반 polling으로 내려갑니다.

### 왜 FIFO/interrupt를 우선했는가
- userspace polling의 샘플 지터를 줄이기 위해
- 여러 샘플을 한 번에 읽어 타이밍 품질을 높이기 위해
- 프레임 사이 고주파 진동을 적분할 때 위상 오차를 줄이기 위해

## 3. LK 추정
LK는 `goodFeaturesToTrack -> calcOpticalFlowPyrLK -> estimateAffinePartial2D(RANSAC)` 순서입니다.

출력:
- `dx`
- `dy`
- `da`
- feature/inlier/confidence

이 구현에서 LK의 주 역할:
- translation 메인 추정
- visual confidence 제공
- 약한 visual rotation anchor 제공

## 4. Gyro 추정
gyro는 각속도 샘플을 `Quaternion`으로 적분합니다.

```text
q(t) = integral(gyro_rad_s, dt)
```

프레임 `k-1`과 `k` 사이의 물리 회전 delta를 구한 뒤, 그 delta에서 high-pass 성분만 뽑아 correction으로 사용합니다.

```text
delta_raw = q_prev^-1 * q_curr
delta_hp  = delta_raw - LPF(delta_raw)
```

### 왜 high-pass를 쓰는가
- RC카 마운트의 고주파 잔떨림을 적극적으로 잡기 위해
- 의도적인 큰 회전까지 계속 붙잡아 검은 여백이 커지는 것을 막기 위해
- pure gyro drift가 장기적으로 커지는 문제를 줄이기 위해

## 5. Fusion 구조
채택 구조는 아래입니다.

```text
translation = LK main
rotation    = gyro high-pass + weak visual anchor
controller  = turn-follow state machine
```

정확히는 다음 homography를 합성합니다.

```text
H_total = H_trans_lk * H_visual_anchor * H_gyro
```

여기서 중요한 점:
- `LK rotation`과 `gyro rotation`을 그대로 더하지 않습니다.
- gyro는 고주파 rotation 담당입니다.
- LK는 translation 메인 + 저강도 visual anchor입니다.

이렇게 하지 않으면 같은 회전을 이중으로 잡아 과보정이 생길 수 있습니다.

## 6. State Machine
### `STABILIZE`
- 직진 또는 약한 회전
- gyro high-pass correction 적극 사용
- LK translation 정상 사용
- visual anchor 사용

### `TURN_FOLLOW`
- 큰 yaw rate가 일정 프레임 이상 유지
- correction gain을 낮춤
- 화면이 실제 회전을 따라가게 함
- 검은 여백/검은 화면 폭주를 줄이는 모드

### `RECOVER`
- 회전 종료 직후
- stabilization gain을 서서히 복귀
- 갑작스런 recenter jump를 막음

## 7. Crop Budget Protection
보정 homography가 요구하는 crop 양을 계산해서 예산을 넘으면 correction 자체를 줄입니다.

```text
if required_crop > crop_budget:
    scale_down(rotation + translation)
```

그 뒤 최종 출력에는 고정 center crop을 적용해 검은 가장자리를 숨깁니다.

이 로직의 목적:
- 큰 회전에서 검정 화면만 보이는 상황 완화
- “무조건 고정”보다 자연스러운 follow 우선

## calibration 절차
1. `build/calib` 실행
2. 시작 직후 2.5초 정도 정지 상태 유지
3. 이후 yaw 중심으로 좌우 회전 움직임을 약 8초간 줌
4. tool이:
   - bias 측정
   - coarse/fine offset sweep
   - yaw 축/sign 힌트 출력
   - `config_local.ini` 저장

저장 항목:
- `calib.bias_x`
- `calib.bias_y`
- `calib.bias_z`
- `calib.imu_offset_ms`
- `calib.ts_source`
- `calib.last_calibration`

주의:
- `bias_x/y/z`는 현재 구현에서 raw gyro count 기준입니다.
- 실제 yaw 축/sign은 하드웨어 장착 방향에 따라 바뀔 수 있습니다.

## `config_local.ini` 설명
### `[camera]`
- `width`, `height`, `fps`
- `frame_duration_us`
- `exposure_us`
- `flip`
- `hfov_deg`, `vfov_deg`
- `libcamera_xrgb`

### `[imu]`
- `bus`, `addr`
- `int_pin_wpi`
- `target_hz`
- `use_fifo`
- `gyro_sensitivity`
- `axis_roll/pitch/yaw`
- `sign_roll/pitch/yaw`

### `[eis]`
- LK:
  - `lk_max_features`
  - `lk_min_features`
  - `lk_min_inliers`
  - `lk_quality`
  - `lk_min_dist`
  - `lk_ransac_thresh`
  - `lk_confidence_gate`
  - `lk_translation_alpha`
  - `lk_translation_max_corr_px`
  - `lk_translation_turn_scale`
  - `lk_rotation_anchor_alpha`
  - `lk_rotation_gain`
- gyro:
  - `gyro_gain_roll/pitch/yaw`
  - `gyro_max_roll/pitch/yaw_deg`
  - `gyro_hp_lpf_alpha`
  - `gyro_hp_gain_roll/pitch/yaw`
  - `gyro_large_rot_thresh_deg`
  - `gyro_large_rot_gain_scale`
- controller:
  - `crop_budget_percent`
  - `turn_enter_yaw_rate_dps`
  - `turn_exit_yaw_rate_dps`
  - `turn_hold_frames`
  - `recover_frames`
  - `turn_follow_correction_scale`
  - `debug_overlay`

### `[rtsp]`
- `port`
- `path`
- `raw_path`
- `bitrate`
- `iframe_period`

### `[mqtt]`
- `host`
- `port`
- `keepalive_sec`
- `topic`

### `[calib]`
- calibration 결과 + sweep 파라미터

## 기대 효과
### 바로 기대할 수 있는 것
- `exposure_us` 단축으로 blur 감소
- `SensorTimestamp` 기반 프레임 시각 확보
- gyro high-pass 보조로 잔떨림 완화 가능성 증가
- 큰 회전 시 `TURN_FOLLOW`로 검은 여백 폭주 완화

### 여전히 남는 한계
- IMX219 rolling shutter 왜곡
- 매우 강한 고주파 진동에서 광학 blur 자체는 복원 불가
- 하드웨어 실기 튜닝 없이는 gain/threshold 최적값 보장 불가

## 구조 A와의 차이
구조 A:

```text
gyro = rotation 전담
LK   = dx/dy 전담
```

이 구조는 직관적이지만 실제로는 다음 문제가 있습니다.
- LK의 `dx/dy`에도 rotation/parallax가 섞임
- gyro rotation과 LK rotation이 완전히 분리되지 않음
- 큰 회전에서 검은 여백 억제 정책이 별도로 필요함

채택 구조:

```text
LK main + gyro high-pass assist + turn-follow
```

채택 이유:
- 현재 환경에서 LK가 이미 일정 수준 검증됨
- gyro는 고주파 jitter와 turn detection에 더 강함
- 같은 회전을 이중 적용하지 않도록 분리하기 쉬움
- RC카의 “직진 안정화 + 회전 시 자연스럽게 따라가기” 요구와 더 잘 맞음

## 사용 예
### 1. calibration
```bash
mkdir -p build
cd build
cmake ..
make -j
sudo ./calib
```

### 2. runtime
```bash
cd build
sudo ./main
```

### 3. RTSP 확인
```text
rtsp://<PI_IP>:8555/cam
rtsp://<PI_IP>:8555/raw
```
