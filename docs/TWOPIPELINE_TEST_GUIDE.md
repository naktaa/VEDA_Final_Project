# Two-Pipeline 테스트 가이드

## 기본 개념
- `RTSP /raw`, `HTTP /stream.mjpg`: 원본 비교용
- `RTSP /cam`, `HTTP /stream_display.mjpg`: 보정 + 표시용 후처리 화면
- 조정 우선순위:
  1. `exposure_us`
  2. `tracking_clahe_clip`
  3. `display_gain`
  4. `display_gamma`
  5. `display_denoise_strength`

## 시작값
- `exposure_us=10000`
- `tracking_clahe_clip=2.0`
- `display_gain=1.10`
- `display_gamma=1.15`
- `display_denoise_strength=0.0`

## 테스트 순서
1. 같은 구간을 `정지`, `직진`, `큰 회전`으로 나눠 반복 테스트
2. 항상 `raw`와 `display`를 같이 비교
3. 한 번에 하나의 파라미터만 변경
4. `display`가 아니라 먼저 `raw blur`와 `보정 안정감`을 기준으로 판단

## 파라미터 방향
### `exposure_us`
- 기본 시작: `10000`
- 밝고 blur가 남으면: `8000`
- 너무 어둡거나 LK가 불안정하면: `10000` 또는 `12000`
- `6000` 이하는 추가 조명 없으면 기본 권장 안 함

### `tracking_clahe_clip`
- 기본: `2.0`
- 어둡고 feature가 약하면: `2.5`
- 노이즈가 과하게 살아나면: `1.5 ~ 2.0`

### `display_gain`
- 기본: `1.10`
- 화면이 어두우면: `1.15 ~ 1.20`
- 너무 올리면 노이즈가 바로 거칠어짐

### `display_gamma`
- 기본: `1.15`
- 어두운 영역만 조금 더 띄우고 싶으면: `1.20 ~ 1.25`
- 너무 높이면 화면이 뜨고 뿌옇게 보일 수 있음

### `display_denoise_strength`
- 기본: `0.0`
- 정말 필요할 때만: `0.03 ~ 0.05`
- 기본은 끄고 시작하는 게 안전함

## 판단 기준
- `tracking` 조정은 보정 성능에 영향
- `display` 조정은 체감 화질에 영향
- `display`만 좋아지고 떨림 체감이 그대로면 정상
- `raw blur`가 심하면 display 후처리로 해결 안 됨

## 추천 운용
- 밝은 환경: `8000 ~ 10000us`
- 보통 실내: `10000us`
- 어두운 실내: `10000 -> 12000us`까지 타협, 그 이상은 조명 추가를 먼저 검토
