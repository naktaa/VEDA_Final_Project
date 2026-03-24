# VEDA RC Auto 브랜치

이 브랜치는 RC카 자동주행 실행부만 담당합니다. 서버가 발행하는 `wiserisk/p1/pose`를 받아서 목표점 추종을 수행하고, UI에는 `test/mqtt/Tank_status` 형식의 `wiserisk/rc/status`만 발행합니다.

## 실행 파일 구성

- `auto_main`
  - MQTT 연결
  - goal / pose / safety 구독
  - 제어 루프 실행
  - `wiserisk/rc/status` 발행
- `config/rc_control.ini`
  - 제어 gain, 모터 파라미터 설정

## 내부 모듈 구조

- `auto_main.cpp`
  - 얇은 엔트리 포인트
- `src/auto_app.cpp`
  - 인자 파싱, 시그널 처리, 실행 orchestration
- `src/rc_control_node.cpp`
  - MQTT 콜백, 상태기, 제어 루프
- `src/rc_config.cpp`
  - `rc_control.ini` 파싱
- `src/rc_json_utils.cpp`
  - goal / pose / safety payload 파싱
- `src/rc_motor_driver.cpp`
  - wiringPi 기반 모터 출력
- `src/rc_status_types.cpp`
  - `Tank_status` 고정 payload 직렬화

## MQTT 인터페이스

- 구독: `wiserisk/rc/goal`
- 구독: `wiserisk/p1/pose`
- 구독: `wiserisk/rc/safety`
- 발행: `wiserisk/rc/status`
- LWT: `wiserisk/rc/status`에 offline 상태 retained 발행

## 상태 필드 매핑

- `connected`: MQTT 연결 상태
- `mode`: goal이 있으면 `auto`, 없으면 `idle`
- `mission`: goal이 있으면 `goal_tracking`, 없으면 `none`
- `speed`: 현재 명령 선속도 `m/s`
- `x`, `y`: 최신 pose 좌표 `m`
- `heading`: 최신 pose yaw를 `deg`로 변환
- `comm_state`: `connected` / `disconnected`
- `robot_state`: `WAIT_INPUT`, `TRACKING`, `ROTATE`, `REACHED`, `SAFE_STOP`, `POSE_TIMEOUT`, `offline`
- `data_period`: `50ms`
- `target`: goal이 있으면 `{x,y,z=-1.0}`, 없으면 placeholder
- `battery`: `-1.0` placeholder 유지

`battery`, `z`, `task_*`, `motors`는 실제 데이터가 없으면 placeholder 또는 omission 정책을 유지합니다.

## 설정 정책

meter 기반 키를 우선 사용합니다.

- `control.max_speed_mps`
- `control.tolerance_m`
- `motor.track_width_m`
- `motor.wheel_max_speed_mps`
- `motor.speed_deadband_mps`

기존 `*_cm*` 키도 한 번은 읽어서 meter로 변환하고 warning을 출력합니다.

## 빌드

필수:

- `libmosquitto`
- `pthread`
- `CMake 3.16+`
- 선택: `wiringPi`

```bash
cmake -S . -B build
cmake --build build --target auto_main -j$(nproc)
```

## 실행

```bash
./build/auto_main \
  192.168.100.10 \
  1883 \
  wiserisk/rc/goal \
  wiserisk/p1/pose \
  wiserisk/rc/safety \
  wiserisk/rc/status \
  ./config/rc_control.ini
```

인자 순서:

1. `mqtt_host`
2. `mqtt_port`
3. `goal_topic`
4. `pose_topic`
5. `safety_topic`
6. `status_topic`
7. `ini_path`

## 빠른 확인

```bash
mosquitto_sub -h 192.168.100.10 -t wiserisk/rc/status -v
```

UI에서 우선 확인할 필드:

- `connected`
- `mode`
- `mission`
- `speed`
- `x`
- `y`
- `heading`
- `comm_state`
- `robot_state`
- `data_period`
- `target`
