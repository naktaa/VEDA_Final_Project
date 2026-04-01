# VEDA 아키텍처

## 개요

VEDA는 MQTT를 공통 제어/상태 버스로 사용하고, 영상 스트림은 RTSP 또는 HTTP로 따로 전달하는 구조입니다.

## 데이터 흐름

1. `server/`는 CCTV RTSP를 읽어 `wiserisk/p1/pose`, `wiserisk/map/H_img2world`, `wiserisk/map/graph`를 발행합니다.
2. `tank/`는 RC 탱크 영상과 제어를 담당하고 `wiserisk/rc/*` 토픽으로 상태/명령을 주고받습니다.
3. `ruview/`는 ESP32-S3에서 Wi-Fi CSI 또는 테스트 UDP 트래픽을 송신합니다.
4. `qt/`는 `wiserisk/#`를 구독하고, CCTV/탱크 스트림과 MQTT 이벤트를 하나의 관제 UI로 합칩니다.

## 모듈별 책임

- `server/`
  - pose 추정
  - homography/map 배포
  - 카메라/바닥 캘리브레이션 도구 제공
- `tank/`
  - 주행 제어 우선순위 `Qt = controller > auto`
  - IMU/LK 기반 하이브리드 EIS
  - RTSP 송출, MJPEG/tiltVR, PTZ
- `ruview/`
  - ESP-IDF 기반 보드 설정
  - UDP target IP/port, node ID를 `menuconfig`로 관리
- `qt/`
  - 이벤트 리스트, 맵 오버레이, RC 제어, clip popup
  - 관제용 런타임 설정은 `config/runtime.ini` 또는 `config/runtime.ini.example`

## 설정 원칙

- 환경 의존 값은 각 모듈 설정 파일로 분리합니다.
- 루트 `CMake`는 Linux 네이티브 모듈인 `server/`, `tank/`만 묶습니다.
- `qt/`, `ruview/`는 플랫폼별 독립 빌드를 유지합니다.
