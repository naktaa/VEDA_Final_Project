# 빌드와 실행

## 1. Linux 통합 빌드

```bash
cmake -S . -B build
cmake --build build -j
```

옵션:

- `-DBUILD_SERVER=OFF`
- `-DBUILD_TANK=OFF`
- `-DBUILD_LEGACY_VARIANTS=ON`

## 2. Server 실행

서버 설정은 `server/config/server.yaml.example`을 참고해서 `build/server/config/server.yaml`로 복사해 사용합니다.

```bash
./build/server/calib_camera <체스보드_이미지_폴더> 8 6 3.0 ./build/server/config/camera.yaml
./build/server/calib_aruco_homography ./build/server/config/H_img2world.yaml
./build/server/main
```

## 3. Tank 실행

탱크 실행 파일 기준으로 `config_template.ini`, `web/`, `certs/`가 `build/tank/`에 복사됩니다.

```bash
./build/tank/calib
./build/tank/main
```

`config_local.ini`가 없으면 `build/tank/config_template.ini`를 바탕으로 생성됩니다.

## 4. Qt 빌드

```bash
cmake -S qt -B build/qt -DOpenCV_DIR=... -DGSTREAMER_ROOT=... -DVCPKG_ROOT=...
cmake --build build/qt --config Release
```

런타임 설정:

- `build/qt/<Config>/config/runtime.ini`
- 없으면 `runtime.ini.example`를 fallback으로 사용

## 5. RuView 빌드

```bash
idf.py -C ruview menuconfig
idf.py -C ruview build
idf.py -C ruview -p <PORT> flash monitor
```

`menuconfig`에서 아래 값을 맞춥니다.

- Wi-Fi SSID / Password
- RX/TX UDP target IP
- RX/TX UDP target port
- RX/TX node ID
