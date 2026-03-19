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
./mqtt_tank_drive
```

Default MQTT:
- Host: `192.168.100.10`
- Port: `1883`
- Topic: `wiserisk/rc/control`

Default RTSP output:
- URL: `rtsp://<PI_IP>:8555/cam`
- Format: `640x480 @ 20fps`, rotate-180 flip, H.264 hardware encoding (`v4l2h264enc`)
- Low-latency: leaky queues enabled, I-frame period set to 20

## Optional RTSP Args
```bash
--no-rtsp
--rtsp-port 8555
--rtsp-path /cam
--rtsp-launch "( libcamerasrc ! video/x-raw,width=640,height=480,framerate=20/1 ! videoflip method=rotate-180 ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! videoconvert ! video/x-raw,format=I420 ! v4l2h264enc extra-controls=\"controls,video_bitrate=1500000,h264_i_frame_period=20\" ! video/x-h264,level=(string)4,profile=(string)baseline ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! rtph264pay name=pay0 pt=96 config-interval=1 )"
```

## Client low-latency tips
- VLC: lower Network caching (e.g. `100ms` or lower).
- Prefer UDP transport when possible to reduce end-to-end delay.

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
