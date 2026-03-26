# VEDA RC Auto 브랜치

이 브랜치는 RC카 자동주행 실행부만 담당합니다. 서버가 발행하는 `wiserisk/p1/pose`를 받아서 목표점 추종을 수행하고, UI에는 `test/mqtt/Tank_status` 형식의 `wiserisk/rc/status`만 발행합니다.

## 실행 파일 구성

- `main`
  - MQTT 연결
  - goal / pose / safety 구독
  - 제어 루프 실행
  - `wiserisk/rc/status` 발행
- `config/rc_control.template.ini`
  - 추적되는 템플릿 설정 파일
- `config/rc_control.ini`
  - 첫 실행 때 템플릿에서 복사되어 생성되는 로컬 설정 파일
  - git에는 올라가지 않음
  - 제어 튜닝값만 수정

## 내부 모듈 구조

- `auto_main.cpp`
  - 얇은 엔트리 포인트
- `src/auto_app.cpp`
  - 고정 경로 설정, 시그널 처리, 실행 orchestration
- `src/rc_control_node.cpp`
  - MQTT 콜백, 상태기, 제어 루프
- `src/rc_config.cpp`
  - `rc_control.ini` 생성 보조 및 파싱
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
- `speed`: 현재 명령 선속도 `cm/s`
- `x`, `y`: 최신 pose 좌표 `cm`
- `heading`: 최신 pose yaw를 `deg`로 변환
- `comm_state`: `connected` / `disconnected`
- `robot_state`: `WAIT_INPUT`, `TRACKING`, `ROTATE`, `REACHED`, `SAFE_STOP`, `POSE_TIMEOUT`, `offline`
- `data_period`: `50ms`
- `target`: goal이 있으면 `{x,y,z=-1.0}`, 없으면 placeholder
- `battery`: `-1.0` placeholder 유지

`battery`, `z`, `task_*`, `motors`는 실제 데이터가 없으면 placeholder 또는 omission 정책을 유지합니다.

## 설정 정책

고정 계약은 코드에 둡니다.

- MQTT broker 기본값: `192.168.100.10:1883`
- 토픽: `wiserisk/rc/goal`, `wiserisk/p1/pose`, `wiserisk/rc/safety`, `wiserisk/rc/status`
- status publish 주기: `50ms`
- payload 형식: `Tank_status` 고정

로컬 튜닝값만 `rc_control.ini`에서 조정합니다.

- `control.k_linear`
- `control.k_yaw`
- `control.max_speed_cmps`
- `control.max_yaw_rate_rps`
- `control.tolerance_cm`
- `motor.track_width_cm`
- `motor.wheel_max_speed_cmps`
- `motor.speed_deadband_cmps`
- `motor.pwm_min_effective`
- `motor.pwm_max`

기존 `*_m*`, `*_mps*` 키도 한 번은 읽어서 centimeter 기반 값으로 변환하고 warning을 출력합니다.

## 빌드

필수:

- `libmosquitto`
- `pthread`
- `CMake 3.16+`
- 선택: `wiringPi`

```bash
mkdir -p build
cd build
cmake ..
make -j
```

## 실행

```bash
sudo ./main
```

동작 방식:

1. `build/main`은 자동으로 상위 디렉터리의 `config/rc_control.ini`를 찾습니다.
2. 파일이 없으면 `config/rc_control.template.ini`를 복사해서 `config/rc_control.ini`를 만들고 종료합니다.
3. 사용자는 `config/rc_control.ini`만 수정한 뒤 다시 `sudo ./main`으로 실행하면 됩니다.

즉 실행 인자는 사용하지 않고, 토픽이나 브로커 주소도 ini에서 바꾸지 않습니다.

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
