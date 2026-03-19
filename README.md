# Tank MQTT Drive

`wiserisk/rc/control` MQTT 토픽을 받아 탱크 주행(`drive`)만 제어하는 최소 프로젝트입니다.

## Build
```bash
mkdir -p build
cd build
cmake ..
make -j
```

필수 라이브러리:
- `wiringPi`
- `libmosquitto` (`libmosquitto-dev`)

## Run
```bash
./mqtt_tank_drive --host 127.0.0.1 --port 1883 --topic wiserisk/rc/control
```

## Supported MQTT Commands
Payload 예시:
```json
{
  "type": "tank_control",
  "target": "tank",
  "group": "drive",
  "command": "forward",
  "active": true,
  "source": "qt_main_window",
  "ts_ms": 1773890000000
}
```

처리 조건:
- `type == "tank_control"`
- `target == "tank"`
- `group == "drive"`

지원 `command`:
- `forward`
- `backward`
- `turn_left`
- `turn_right`

동작 규칙:
- `active=true`: 해당 동작 시작
- 동일 `command`의 `active=false`: 정지
- `group=ptz`는 무시
