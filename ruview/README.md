# RuView ESP32-S3 Module

ESP-IDF 기반 RuView 펌웨어 프로젝트입니다. 현재 저장소 범위에서는 ESP32-S3 Wi-Fi/CSI UDP 송신 쪽만 포함합니다.

## 구조

- `CMakeLists.txt`
- `sdkconfig.defaults`
- `main/`
  - `rx_main.c`
  - `tx_main.c`
  - `station_example_main.c`
  - `Kconfig.projbuild`
  - `idf_component.yml`

## 빌드

```bash
idf.py -C ruview menuconfig
idf.py -C ruview build
idf.py -C ruview -p <PORT> flash monitor
```

## menuconfig 항목

- Wi-Fi SSID / Password
- RX UDP target IP / port / node ID
- TX UDP target IP / port / node ID

## 엔트리포인트

- 기본 빌드 소스: `main/rx_main.c`
- 보조/실험용 소스: `main/tx_main.c`, `main/station_example_main.c`

## 메모

- IDE 생성 파일과 `sdkconfig` 스냅샷은 정리하고 제거했습니다.
- 실제 배포 시에는 `menuconfig`에서 보드별 네트워크 값과 node ID를 명시적으로 맞추는 전제를 둡니다.
