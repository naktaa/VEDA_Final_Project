# EIS Strong 테스트 가이드

## 무엇을 먼저 볼까
- `robust LK` 적용 뒤 헛보정이 줄었는지
- `weak frame`에서 화면이 덜 끌려가는지
- 큰 회전 후 `follow/recover`가 덜 늦는지

## 테스트 순서
1. 같은 구간을 `정지`, `직진`, `큰 회전`으로 나눠 반복 테스트
2. 먼저 구조 효과를 확인
3. 그 다음 아래 파라미터만 하나씩 조정

## 우선 조정할 파라미터
### `lk_confidence_gate`
- 기본: `0.55`
- weak frame이 너무 자주 나와 보정이 자주 끊기면: `0.50`
- 헛보정이 아직 많으면: `0.60`

### `lk_rotation_gain`
- 기본: `0.18`
- 화면이 괜히 비틀리면: `0.12 ~ 0.15`
- 회전 보정이 너무 약하면: `0.20 ~ 0.25`

### `turn_follow_correction_scale`
- 기본: `0.25`
- 회전 중 너무 풀려서 덜 안정적이면: `0.30 ~ 0.40`
- 회전을 더 자연스럽게 따라가게 하려면: `0.15 ~ 0.20`

### `recover_frames`
- 기본: `12`
- 회전 후 너무 늦게 따라오면: `8 ~ 10`
- 더 부드럽게 복귀시키고 싶으면: `14 ~ 16`

### `exposure_us`
- 기본 시작: `10000`
- 밝고 blur가 남으면: `8000`
- 어두워서 LK가 약하면: `10000 ~ 12000`

## 판단 기준
- `lk_confidence_gate`: weak frame 민감도 조정
- `lk_rotation_gain`: 회전 과보정/부족 조정
- `turn_follow_correction_scale`: 회전 중 안정화와 자연스러움 균형
- `recover_frames`: 회전 종료 후 복귀 속도
- `exposure_us`: LK 입력 품질 자체 조정

## 크게 안 봐도 되는 것
- `gyro_gain_*`
- `gyro_hp_*`
- `gyro_large_rot_*`

이 브랜치는 gyro 적극 활용이 아니라 `LK 강건화`가 핵심이라서,
먼저 위 5개만 보는 게 효율적입니다.

## 추천 시작점
- `lk_confidence_gate=0.55`
- `lk_rotation_gain=0.18`
- `turn_follow_correction_scale=0.25`
- `recover_frames=10~12`
- `exposure_us=10000`
