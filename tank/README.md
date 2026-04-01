# Tank Module

라즈베리파이 RC 탱크 통합 런타임입니다. 주행 제어, 하이브리드 EIS, RTSP, tiltVR HTTP 서버를 포함합니다.

## 구조

- `src/`, `include/`
- `config/config_template.ini`
- `docs/`
- `web/tilt_vr`
- `scripts/gyro_axis_plot.py`

## 빌드

```bash
cmake -S . -B build
cmake --build build --target main calib -j
```

필요 시:

```bash
cmake -S . -B build -DBUILD_LEGACY_VARIANTS=ON
cmake --build build -j
```

출력:

- `build/tank/main`
- `build/tank/calib`
- `build/tank/main_raw`
- `build/tank/lkonly`

## 실행

`build/tank/` 아래로 템플릿 설정과 웹 자산이 복사됩니다.

```bash
./build/tank/calib
./build/tank/main
```

`config_local.ini`가 없으면 `build/tank/config_template.ini`를 기준으로 생성합니다.

## 설정

주요 파일:

- 소스 템플릿: `tank/config/config_template.ini`
- 런타임 템플릿 복사본: `build/tank/config_template.ini`
- 로컬 설정: `build/tank/config_local.ini`

기본 `web_root`, TLS 경로는 실행 파일 기준 상대 경로입니다.

- `web/tilt_vr`
- `certs/tiltvrsvr-cert.pem`
- `certs/tiltvrsvr-key.pem`

## 문서

- `docs/HYBRID_EIS_FRAMEWORK.md`
- `docs/TILTVR_HTTPS.md`
- `docs/TWOPIPELINE_TEST_GUIDE.md`
