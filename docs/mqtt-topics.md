# MQTT 토픽 계약

## Server 발행

- `wiserisk/p1/pose`
- `wiserisk/map/H_img2world`
- `wiserisk/map/graph`

## Tank 구독/발행

- 구독: `wiserisk/rc/control`
- 구독: `wiserisk/rc/goal`
- 구독: `wiserisk/p1/pose`
- 구독: `wiserisk/rc/safety`
- 발행: `wiserisk/rc/status`

## Qt 구독/발행

- 구독: `wiserisk/#`
- 발행: `wiserisk/commands/vision_pose_bridge`
- 발행: `wiserisk/rc/control`
- 발행: `wiserisk/rc/goal`
- 발행: `wiserisk/calib/homography`
- 발행: `wiserisk/clips/forward`

## 비고

- 1차 통합에서는 토픽 이름을 바꾸지 않았습니다.
- 브로커 주소와 포트는 각 모듈 설정 파일에서 관리합니다.
