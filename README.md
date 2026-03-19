# EIS 사용법

## 빌드
```bash
mkdir build
cd build
cmake ..
make
```

## 실행
```bash
./eis [옵션]
```

기본 RTSP 포트는 `8555` 입니다.
현재 코드에서는 테스트 환경에 맞춰 입력 영상을 **상하 반전**합니다.
현재 기본 gyro 튜닝은 **RC카 고주파 떨림 우선** 기준으로 맞춰져 있습니다.
- 기본 gyro warp mode: `highpass`
- 기본 yaw gain: 보수적으로 낮춤 (`0.45`)

### 옵션
```text
--mode {lk|gyro|hybrid}
--imu-offset-ms <double>
--offset-sweep
--overlay {0|1}
--log-every-frames <N>
--ts-source {auto|sensor|pts|arrival}
```

### 실행 중 키 입력
```text
1 = LK only
2 = Gyro debug
3 = LK + Gyro hybrid
4 = raw 영상만 송출
5 = cam 영상만 송출
```

## VLC로 보기
```text
rtsp://<장치_IP>:8555/raw   (원본)
rtsp://<장치_IP>:8555/cam   (보정)
```

## 타임스탬프 정렬 검증 절차
1. `./eis --mode gyro --offset-sweep --overlay 1`
2. `[IMU] Ready` 직후 6~8초 동안 yaw로 2~3회 좌우 회전
3. 콘솔 로그에서 `[SYNC] offset`, `[CMP] lk_da vs gyro_dyaw`, `corr` 확인
4. 결과 오프셋을 `--imu-offset-ms <값>`으로 고정 후 재실행
5. 실행 중 `3` 키로 hybrid 전환, LK only 대비 차이 확인

## Drift / Bias 처리
- 시작 시 정지 상태에서 gyro bias 자동 보정
- bias 제거 후 적분
- hybrid에서는 고주파 회전 성분만 보조로 사용

## RC카 현장 테스트 권장 순서
1. gyro 단독 확인
```bash
./eis --mode gyro --profile debug --overlay 1 --log-every-frames 10
```
2. 타임스탬프 정렬 확인
```bash
./eis --mode gyro --offset-sweep --profile debug --overlay 1 --log-every-frames 10
```
3. hybrid 확인
```bash
./eis --mode hybrid --profile debug --overlay 1 --log-every-frames 10
```

권장 확인 로그:
- `[TS] source=SENSOR`
- `[SYNC] offset=...`
- `[CMP] ... corr=...`
- `[HIGHPASS] ... corr(deg)=...`
- `[PROTECT] large_rot=... gain_scaled=...`
