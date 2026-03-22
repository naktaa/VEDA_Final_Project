# Tank MQTT Drive + RTSP + tiltVR

이 프로젝트는 다음 기능을 한 프로세스에서 함께 제공합니다.
- MQTT 기반 탱크 주행 제어 (`drive` 그룹)
- MQTT 기반 팬/틸트 PTZ 버튼 제어 (`ptz` 그룹)
- RTSP 영상 송출 (`rtsp://<PI_IP>:8555/cam`)
- HTTP/MJPEG + 모바일 VR split-view (`http://<PI_IP>:8000/web/`)
- 휴대폰 IMU 기반 UART 팬/틸트 제어, MQTT PTZ fallback 지원
- 카메라 캡처 브리지: `appsink(libcamerasrc) -> appsrc(RTSP factory)`

## 빌드
```bash
mkdir -p build
cd build
cmake ..
make -j
```

필요 패키지:
- `wiringPi`
- `libmosquitto-dev`
- `libgstreamer1.0-dev`
- `libgstrtspserver-1.0-dev`
- `cpp-httplib` 헤더 (`httplib.h`)
- `nlohmann-json3-dev`
- `libopencv-dev`
- `libcamerasrc`, `v4l2h264enc`를 포함한 GStreamer 플러그인

## 실행
기본 실행은 아래처럼 옵션 없이 사용하면 됩니다.

```bash
./main
```

호환용 별칭:
- 빌드 후 `./mqtt_tank_drive`도 같이 생성됩니다

## 기본 설정
기본 MQTT:
- Host: `192.168.100.10`
- Port: `1883`
- Topic: `wiserisk/rc/control`

기본 PTZ 시리얼:
- Device: `/dev/serial0`
- Baud: `115200`
- Pan/Tilt ID: `1 / 2`
- Center: `1800 / 2400`

기본 RTSP 출력:
- URL: `rtsp://<PI_IP>:8555/cam`
- 포맷: `640x480 @ 20fps`
- 180도 회전 적용
- H.264 하드웨어 인코딩 사용 (`v4l2h264enc`)
- 저지연 설정: leaky queue, I-frame period `10`

기본 HTTP/VR 출력:
- Web UI: `http://<PI_IP>:8000/web/`
- MJPEG: `http://<PI_IP>:8000/stream.mjpg`
- Health: `http://<PI_IP>:8000/health`
- 최신 IMU/PTZ 상태: `http://<PI_IP>:8000/imu/latest`
- MJPEG용 JPEG 인코딩은 웹 스트림 클라이언트가 실제로 연결됐을 때만 동작합니다

## 선택 옵션
```bash
--no-rtsp
--rtsp-port 8555
--rtsp-path /cam
--rtsp-launch "( appsrc name=stabsrc is-live=true format=time do-timestamp=true block=false ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! videoflip method=rotate-180 ! videoconvert ! video/x-raw,format=I420 ! v4l2h264enc extra-controls=\"controls,video_bitrate=1500000,h264_i_frame_period=20\" ! video/x-h264,level=(string)4,profile=(string)baseline ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! rtph264pay name=pay0 pt=96 config-interval=1 )"
--no-http-vr
--http-port 8000
--serial-dev /dev/serial0
--serial-baud 115200
```

## 저지연 사용 팁
- VLC를 쓸 경우 Network caching 값을 낮게 설정하는 편이 좋습니다. 예: `100ms` 이하
- 가능하면 UDP 전송을 우선 사용하세요
- 2.4GHz 환경에서는 RTSP와 MJPEG를 동시에 여러 단말에서 붙이면 지연이 커질 수 있습니다

## MQTT 제어 메시지 형식
예시 payload:

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

수신 조건:
- `type == "tank_control"`
- `target == "tank"`
- `group == "drive"` 또는 `group == "ptz"`

지원 명령:
- `forward`
- `backward`
- `turn_left`
- `turn_right`
- `pan_left`
- `pan_right`
- `tilt_up`
- `tilt_down`

동작 규칙:
- `active=true`: 동작 시작
- 같은 `command`에 대해 `active=false`: 동작 정지
- `group=drive`: 탱크 주행 제어
- `group=ptz`: 팬/틸트 버튼 제어
- 최근 휴대폰 IMU 입력이 들어오면 `300ms` 동안 IMU가 우선합니다
- IMU 입력이 끊기면 다시 MQTT PTZ 제어가 유효해집니다

## HTTP IMU 입력 형식
```json
{
  "type": "imu",
  "t": 1773890000000,
  "pitch": -12.4,
  "roll": 8.1,
  "yaw": 92.0
}
```

포함된 `/web/` 클라이언트는 휴대폰 `deviceorientation` 값을 읽어서 0점 보정 후 `/imu`로 전송합니다.
