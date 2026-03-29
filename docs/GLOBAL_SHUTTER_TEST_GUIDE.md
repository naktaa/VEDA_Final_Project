# Global Shutter 테스트 가이드

## 목적
- `IMX219 rolling shutter`와 `IMX296 global shutter`를 같은 시스템에서 빠르게 비교
- 1차 목표는 `rolling shutter 차이` 확인
- 렌즈/FOV 정보가 없으므로 지금 단계는 `smoke test + A/B 비교 준비`에 집중

## 기본 원칙
- 같은 마운트 위치에서 비교
- 같은 해상도, 같은 FPS로 비교
- 같은 주행 구간을 반복해서 비교
- 처음에는 `raw`를 먼저 비교하고, 그 다음 `EIS`를 비교

## 준비 순서
1. 기존 `config_local.ini` 백업
2. 카메라 교체 후 아래 명령으로 인식 확인
   - `rpicam-hello --list-cameras`
   - 또는 `libcamera-hello --list-cameras`
3. `docs/IMX296_TEST_SNIPPET.ini` 내용을 참고해 `config_local.ini`의 핵심 값만 반영
4. 렌즈 정보가 없으므로 `hfov_deg`, `vfov_deg`는 임시 placeholder로 둠
5. 테스트 시작 전 `rs_mode=off` 유지

## 비교 순서
### 1. Raw 비교
- 경로:
  - `RTSP /raw`
  - 필요 시 웹 raw 경로
- 장면:
  - 정지 상태 잔떨림
  - 직진 고주파 진동
  - 빠른 회전
- 확인할 것:
  - line-wise wobble 감소 여부
  - 화면 휘어짐 감소 여부
  - 빠른 진동에서 모양이 덜 일그러지는지

### 2. EIS 비교
- 경로:
  - `RTSP /cam`
- 확인할 것:
  - LK가 가짜 회전/가짜 translation을 덜 만드는지
  - 회전 중/회전 후 보정이 더 자연스러운지
  - weak frame이나 순간 튐이 줄어드는지

## 설정 시작점
- `width=640`
- `height=480`
- `fps=20`
- `frame_duration_us=50000`
- `exposure_us=8000`부터 시작
- `capture_backend=libcamera`
- `rs_mode=off`

## 내일 렌즈 정보가 오면 할 일
1. 렌즈 초점거리 확인
2. `hfov_deg`, `vfov_deg` 계산 후 반영
3. 필요하면 `exposure_us` 재조정
4. 최종 비교 전에 `calib` 다시 실행해서 `imu_offset_ms` 재설정

## 체크리스트
- 카메라 인식됨
- 같은 해상도/FPS로 비교함
- raw와 EIS를 분리해서 확인함
- 렌즈 정보 전에는 FOV 값이 placeholder임을 인지함
- 최종 결론 전 `calib` 재실행 예정
