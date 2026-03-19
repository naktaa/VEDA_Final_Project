# Tank MQTT Drive + RTSP

Minimal project for:
- MQTT-based tank driving control (`drive` group)
- RTSP video streaming (`rtsp://<PI_IP>:8555/cam`)

## Build
```bash
mkdir -p build
cd build
cmake ..
make -j
```

Required packages:
- `wiringPi`
- `libmosquitto-dev`
- `libgstreamer1.0-dev`
- `libgstrtspserver-1.0-dev`
- GStreamer plugins including `libcamerasrc` and an H.264 encoder (for default launch)

## Run
```bash
./mqtt_tank_drive --host 127.0.0.1 --port 1883 --topic wiserisk/rc/control
```

Default RTSP output:
- URL: `rtsp://<PI_IP>:8555/cam`

## Optional RTSP Args
```bash
--no-rtsp
--rtsp-port 8555
--rtsp-path /cam
--rtsp-launch "( libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 ! rtph264pay name=pay0 pt=96 config-interval=1 )"
```

## Supported MQTT Payload
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

Accepted filters:
- `type == "tank_control"`
- `target == "tank"`
- `group == "drive"`

Supported commands:
- `forward`
- `backward`
- `turn_left`
- `turn_right`

Behavior:
- `active=true`: start motion
- matching `command` + `active=false`: stop motion
- `group=ptz` is ignored
