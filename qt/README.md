# Qt Module

Windows 기반 관제 클라이언트입니다. CCTV, Tank 영상, MQTT 이벤트, 맵 오버레이, clip popup을 한 화면으로 합칩니다.

## 구조

- `src/`, `include/`, `ui/`
- `resources/images/`
- `resources/app_resources.qrc`
- `config/runtime.ini.example`
- `docs/qt_control_system_integrated_documentation.md`

## 빌드

```bash
cmake -S qt -B build/qt -DOpenCV_DIR=... -DGSTREAMER_ROOT=... -DVCPKG_ROOT=...
cmake --build build/qt --config Release
```

## 런타임 설정

관제 환경 의존 값은 `config/runtime.ini`에서 읽습니다.

- MQTT broker host/port
- topic filter
- central CCTV RTSP
- tank RTSP
- RuView zone config script path

실행 파일 옆 `config/runtime.ini`가 없으면 `config/runtime.ini.example`를 읽습니다.

## 메모

- UI 상태 저장은 `QSettings`를 유지합니다.
- 오버레이 보정값은 실행 디렉터리의 `config/overlay_calib.ini`에 저장합니다.
