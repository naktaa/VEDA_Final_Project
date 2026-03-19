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
- GStreamer plugins including `libcamerasrc` and `v4l2h264enc` (hardware encoder)

## Run
```bash
./mqtt_tank_drive --host 127.0.0.1 --port 1883 --topic wiserisk/rc/control
```

Default RTSP output:
- URL: `rtsp://<PI_IP>:8555/cam`
- Format: `640x480 @ 20fps`, H.264 hardware encoding (`v4l2h264enc`)

## Optional RTSP Args
```bash
--no-rtsp
--rtsp-port 8555
--rtsp-path /cam
--rtsp-launch "( libcamerasrc ! video/x-raw,width=640,height=480,framerate=20/1 ! videoconvert ! video/x-raw,format=I420 ! v4l2h264enc extra-controls=\"controls,video_bitrate=1500000,h264_i_frame_period=30\" ! video/x-h264,level=(string)4,profile=(string)baseline ! rtph264pay name=pay0 pt=96 config-interval=1 )"
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
