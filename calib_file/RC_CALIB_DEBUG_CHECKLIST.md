# RC 3D ArUco 캘리브레이션/디버그 체크리스트

## 1) 목적
이 문서는 다음을 빠르게 분리 진단하기 위한 실전 체크리스트다.

1. 캘리브레이션 문제인지
2. 메시지(MQTT) 문제인지
3. 제어 로직/하드웨어 매핑 문제인지

핵심 원칙: **통합 실행 전에 단계별로 통과/실패를 분리**한다.

---

## 2) 현재 시스템 요약

1. `calib_aruco_homography.cpp`: 바닥 ArUco 4점으로 `H_img2world` 생성
2. `homography_mqtt_pub.cpp`: `wiserisk/map/H_img2world`, `wiserisk/map/graph` 발행
3. `p1_tracker.cpp`: CCTV + ArUco(21~25)로 `wiserisk/p1/pose` 발행
4. Qt: 목표점 클릭 후 `wiserisk/rc/goal` 발행
5. `rc/rc_control_node.cpp`: goal/pose/safety 구독, 모터 명령 생성

---

## 3) 우선순위 높은 문제 후보

1. **단위계 불일치(cm/m)**
2. **큐브 면별 좌표계 가정(ID/방향) 불일치**
3. **포즈 유실 + 300ms timeout으로 stop-go**
4. **모터 방향/좌우 매핑 반전**
5. **엄격한 JSON 포맷(frame/ts_ms) 불일치**

---

## 4) 단계별 점검 절차 (반드시 순서대로)

## Stage 0. 프로세스/토픽 기본 확인

실행 프로세스가 다 떠 있는지 먼저 확인.

```bash
ps -ef | rg "homography_mqtt_pub|p1_tracker|rc_control_node|mosquitto"
```

통과 기준:
- 필요한 프로세스가 모두 실행 중

실패 시:
- 실행 순서 문제 또는 실행파일/경로 문제부터 해결

---

## Stage 1. MQTT 메시지 스키마 검증 (제일 먼저)

아래 4개 토픽을 각각 실시간 확인.

```bash
mosquitto_sub -h 192.168.100.7 -t wiserisk/rc/goal -v
mosquitto_sub -h 192.168.100.7 -t wiserisk/p1/pose -v
mosquitto_sub -h 192.168.100.7 -t wiserisk/rc/safety -v
mosquitto_sub -h 192.168.100.7 -t wiserisk/rc/status -v
```

통과 기준:
1. goal: `{"x":...,"y":...,"frame":"world","ts_ms":...}`
2. pose: `{"x":...,"y":...,"yaw":...,"frame":"world","ts_ms":...}`
3. safety: `true/false` bool 필드
4. status가 주기적으로 업데이트

실패 시:
- JSON 키 이름 오타, `frame != world`, `ts_ms` 누락, 숫자 타입 불일치 수정

---

## Stage 2. 카메라 내부 파라미터 점검

```bash
cd /home/pi/final_veda_test/map/build
./calib_camera <image_dir> 8 6 3.0 camera.yaml
```

통과 기준:
- RMS가 급격히 크지 않고(일반적으로 낮을수록 좋음), 결과가 재실행 시 크게 흔들리지 않음

실패 시:
- 캘리브레이션 이미지 품질(흔들림/각도 다양성/조명) 재수집

---

## Stage 3. 호모그래피(2D) 점검

```bash
cd /home/pi/final_veda_test/map/build
./calib_aruco_homography ./H_img2world.yaml
```

통과 기준:
- 바닥 4점(10,11,12,13) 재투영 오차가 작고 반복 시 결과가 안정적

실패 시:
- 마커 위치 재배치, 카메라 고정/초점/노출 안정화, 다시 캘리브레이션

---

## Stage 4. 단위계 통일 점검 (매우 중요)

질문:
1. `id2world` 숫자(예: 95, 330)가 m인지 cm인지?
2. `markerSize`, `cubeSize`(예: 0.17)는 m인지?
3. Qt goal 좌표는 어떤 단위인지?

현장 테스트:
- 실제 1m 이동시 pose의 `x/y` 변화량이 **1.0 근처**여야 m 일관성 확보

판정:
- 1m 이동에 100 변하면 cm/m 혼용
- 이 경우 제어기 파라미터(`max_speed`, `tolerance`) 의미가 깨짐

---

## Stage 5. 큐브 면(ID 21~25) 좌표계 점검

`p1_tracker`는 면별로 회전행렬을 하드코딩한다. 물리 부착과 다르면 yaw가 틀어진다.

점검 방법:
1. 한 면(ID 하나)만 정면으로 보이게 고정
2. RC를 시계/반시계로 천천히 회전
3. `yaw`가 예상 방향으로 연속적으로 증가/감소하는지 확인

통과 기준:
- 면 전환 전후로 yaw가 큰 점프 없이 연속

실패 시:
- 면별 ID 배치, 마커 회전(붙인 방향), `R_c_m` 가정 재검증 필요

---

## Stage 6. 포즈 유실/타임아웃 점검

설명:
- `rc_control_node`는 최신 pose 수신 후 300ms 넘으면 `POSE_TIMEOUT`으로 정지

```bash
# rc status 모니터링
mosquitto_sub -h 192.168.100.7 -t wiserisk/rc/status -v
```

통과 기준:
- 주행 중 mode가 `TRACKING` 유지, `POSE_TIMEOUT` 거의 없음

실패 시:
- 카메라 가림/블러/조명/프레임드랍/마커 유실 개선
- timeout 완화(예: 700~1000ms)도 실험

---

## Stage 7. 제어기 드라이런(모터 무부하)

`rc_control_node` 로그에서 `CMD v,w` 및 `L/R pwm,dir` 확인.

통과 기준:
1. 목표가 전방이면 양쪽 FORWARD
2. 좌회전 목표면 좌우 휠 속도 차가 기대 방향
3. 목표 근처에서 속도가 줄고 `REACHED` 진입

실패 시:
- yaw 부호, 목표 좌표계, 좌우 바퀴 방향 매핑 재확인

---

## Stage 8. 모터 하드웨어 매핑 점검

간단 명령에서 바퀴 방향이 기대와 일치하는지 확인.

통과 기준:
- 전진 명령시 실제 전진
- +yaw 명령시 기대한 회전 방향

실패 시:
- IN1/IN2 반전 또는 좌우 채널 교차 가능성 높음

---

## 5) 디버그 포인트 (현장용)

## A. 바로 볼 로그

1. `rc_control_node`:
- `[GOAL]`, `[POSE]`, `[CMD]`, `mode`
- `POSE_TIMEOUT` 빈도

2. `p1_tracker`:
- `[STAT] fps`, `ids_seen`, `id21_25_detect_total`, `pose_pub_total`, `read_fail_streak`

3. MQTT 모니터:
- topic별 payload 지연/누락/형식 오류

---

## B. 디버그 시 반드시 기록할 값

아래를 노트에 매 테스트마다 남긴다.

1. 실행 시간
2. markerSize/cubeSize
3. id2world 단위(m/cm)
4. goal 좌표 단위
5. pose Hz
6. `POSE_TIMEOUT` 발생 횟수
7. 좌/우 회전 테스트 결과(정상/반전)

---

## C. 증상 -> 의심 원인 빠른 매핑

1. 제자리에서 덜덜 멈췄다 출발 반복
- pose dropout + 300ms timeout

2. 목표 반대 방향으로 회전
- yaw 축/부호 또는 모터 방향 반전

3. 목표를 절대 못 찍고 계속 과하게 움직임
- 단위계 혼용(cm/m), tolerance 스케일 불일치

4. 면 바뀔 때 갑자기 방향 튐
- 큐브 면별 좌표계(ID/회전) 설정 불일치

---

## 6) 권장 점검 순서(요약)

1. Stage 1 (메시지 스키마)
2. Stage 4 (단위계)
3. Stage 5 (큐브 면/좌표계)
4. Stage 6 (pose 유실)
5. Stage 7~8 (제어/하드웨어)

이 순서대로 하면 원인 분리가 가장 빠르다.

---

## 7) 필요하면 바로 할 추가 작업

1. `rc_control_node`에 디버그 카운터 추가
- pose 수신 Hz, timeout 카운트, invalid payload 카운트

2. `p1_tracker`에 디버그 추가
- 선택된 marker ID, 면 전환 횟수, yaw jump 크기

3. 파라미터 외부화
- timeout, rotate threshold, pwm_min_effective를 실행 인자로 분리

