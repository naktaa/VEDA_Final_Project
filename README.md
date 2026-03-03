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
./eis
```

실행하면 RTSP 서버가 `8555` 포트로 열립니다.

## VLC로 보기
VLC에서 아래 주소로 접속합니다.

```text
| rtsp://<기기_IP>:8555/raw | 원본 |
| rtsp://<기기_IP>:8555/cam | 보정 |
```

