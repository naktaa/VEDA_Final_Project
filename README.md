# VEDA RC Auto Branch

This branch contains only the RC car auto-drive node and the `wiserisk/rc/status` publisher logic based on the `test/mqtt/Tank_status` payload schema.

## Responsibility

- `rc/rc_control_node.cpp`, `rc/rc_control_node.h`
  - Subscribe to `wiserisk/rc/goal`, `wiserisk/p1/pose`, `wiserisk/rc/safety`
  - Generate motor commands
  - Publish `wiserisk/rc/status`
- `include/rc_status_types.h`, `src/rc_status_types.cpp`
  - Define and serialize the fixed `Tank_status` payload shape
- `config/rc_control.ini`
  - RC control and motor parameters

## MQTT Interface

- Subscribe: `wiserisk/rc/goal`
- Subscribe: `wiserisk/p1/pose`
- Subscribe: `wiserisk/rc/safety`
- Publish: `wiserisk/rc/status`
- LWT: retained offline snapshot on `wiserisk/rc/status`

## Status Mapping

- `connected`: MQTT connection state
- `mode`: `auto` when a goal exists, otherwise `idle`
- `mission`: `goal_tracking` when a goal exists, otherwise `none`
- `speed`: current commanded speed in `m/s`
- `x`, `y`: latest pose in world `m`
- `heading`: latest pose yaw converted to `deg`
- `comm_state`: `connected` / `disconnected`
- `robot_state`: `WAIT_INPUT`, `TRACKING`, `ROTATE`, `REACHED`, `SAFE_STOP`, `POSE_TIMEOUT`, `offline`
- `data_period`: `50ms`
- `target`: goal `(x, y)` with `z = -1.0`, or placeholder `(-1.0, -1.0, -1.0)`
- `battery`: placeholder `-1.0`
- `z`, `task_*`, `motors`: omitted unless real data is available

## Config Policy

The branch uses meter-based RC config keys:

- `control.max_speed_mps`
- `control.tolerance_m`
- `motor.track_width_m`
- `motor.wheel_max_speed_mps`
- `motor.speed_deadband_mps`

Legacy `*_cm*` keys are still accepted once and converted to meters with a warning.

## Build

Prerequisites:

- libmosquitto
- pthreads
- CMake 3.16+
- optional: wiringPi

```bash
cmake -S . -B build
cmake --build build --target rc_control_node -j$(nproc)
```

## Run

```bash
./build/rc_control_node \
  192.168.100.10 \
  1883 \
  wiserisk/rc/goal \
  wiserisk/p1/pose \
  wiserisk/rc/safety \
  wiserisk/rc/status \
  ./config/rc_control.ini
```

## Quick Checks

Use `mosquitto_sub` to inspect the status payload:

```bash
mosquitto_sub -h 192.168.100.10 -t wiserisk/rc/status -v
```

Example expected UI-connected fields:

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
