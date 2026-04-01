# VEDA Final Project

통합 메인 브랜치 기준 저장소입니다. 모듈별로 나뉘어 있던 브랜치를 `server/`, `tank/`, `ruview/`, `qt/`로 정리했고, 루트 문서는 전체 빌드/실행 흐름과 모듈 간 계약만 다룹니다.

## 구조

```text
VEDA_Final_Project/
├─ README.md
├─ docs/
├─ cmake/
├─ server/
├─ tank/
├─ ruview/
└─ qt/
```

## 모듈

- `server/`: CCTV RTSP를 읽어 pose와 map/homography MQTT를 발행하는 중앙 서버
- `tank/`: 라즈베리파이 RC 탱크 런타임, 하이브리드 EIS, RTSP, tiltVR HTTP 서버
- `ruview/`: ESP32-S3 기반 RuView 펌웨어 프로젝트
- `qt/`: Windows Qt 관제 클라이언트

## 빌드 진입점

### Linux 통합 빌드

```bash
cmake -S . -B build
cmake --build build -j
```

생성 실행 파일:

- `build/server/main`
- `build/server/calib_camera`
- `build/server/calib_aruco_homography`
- `build/tank/main`
- `build/tank/calib`
- `build/tank/main_raw` (`-DBUILD_LEGACY_VARIANTS=ON`)
- `build/tank/lkonly` (`-DBUILD_LEGACY_VARIANTS=ON`)

### Windows Qt

```bash
cmake -S qt -B build/qt -DOpenCV_DIR=... -DGSTREAMER_ROOT=... -DVCPKG_ROOT=...
cmake --build build/qt --config Release
```

### ESP-IDF RuView

```bash
idf.py -C ruview build
```

## 문서

- [전체 구조](docs/architecture.md)
- [빌드/실행](docs/build.md)
- [MQTT 토픽 계약](docs/mqtt-topics.md)
- [브랜치 import 내역](docs/import-map.md)

## 모듈별 추가 문서

- [server/README.md](server/README.md)
- [tank/README.md](tank/README.md)
- [qt/README.md](qt/README.md)
- [ruview/README.md](ruview/README.md)
