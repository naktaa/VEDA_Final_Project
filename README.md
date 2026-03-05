# Yahboom Bus Servo Control using Raspberry Pi

Raspberry Pi와 Yahboom PTZ Bus Servo 확장보드를 이용하여 **UART 기반 직렬 통신으로 PAN/TILT 서보를 제어하는 시스템**이다.
일반적인 PWM 서보 제어 방식이 아닌 **Bus Servo Protocol (Serial Communication)**을 사용한다.

---

# 1. System Overview

본 시스템은 Raspberry Pi의 UART 인터페이스를 사용하여 Yahboom Bus Servo를 제어한다.

서보는 **PAN / TILT 구조의 PTZ(팬틸트) 메커니즘**으로 구성되어 있으며,
두 개의 서보를 **동시에 제어**할 수 있다.

```
Raspberry Pi
     │
     │ UART
     │
Yahboom Servo Control Board
     │
     ├── Servo ID 1 → PAN
     │
     └── Servo ID 2 → TILT
```

---

# 2. Hardware Components

| Component                   | Description                |
| --------------------------- | -------------------------- |
| Raspberry Pi 4              | Main controller            |
| Yahboom Bus Servo           | PTZ servo motor            |
| Yahboom Servo Control Board | Bus servo controller       |
| External Power Supply       | Servo power                |
| UART connection             | Raspberry Pi ↔ Servo board |

---

# 3. Wiring

UART 통신을 사용하여 Raspberry Pi와 서보 보드를 연결한다.

| Raspberry Pi Pin | Function |
| ---------------- | -------- |
| GPIO14 (Pin 8)   | UART TX  |
| GPIO15 (Pin 10)  | UART RX  |
| GND              | Ground   |

---

# 4. UART Configuration

Raspberry Pi에서 UART 기능을 활성화한다.

```
sudo raspi-config
```

설정 경로

```
Interface Options
 → Serial Port
```

설정

```
Login shell over serial : No
Serial hardware enable : Yes
```

확인

```
ls -l /dev/serial0
```

예시 출력

```
/dev/serial0 -> ttyS0
```

---

# 5. Software Structure

프로젝트 폴더 구조

```
meri
 ├─ servoserial.py
 ├─ two_servo.py
 └─ README.md
```

---

# 6. Python Dependencies

필요 라이브러리

```
pyserial
RPi.GPIO
```

설치

```
pip3 install pyserial
```

---

# 7. Bus Servo Communication

Yahboom Bus Servo는 **UART 기반 패킷 통신**을 사용한다.

패킷 구조

```
FF FF ID LEN CMD ADDR DATA CHECKSUM
```

예시

```
FF FF 01 07 03 2A 02 BC 00 0A 02
```

| Field    | Description     |
| -------- | --------------- |
| FF FF    | Header          |
| ID       | Servo ID        |
| LEN      | Packet length   |
| CMD      | Command         |
| ADDR     | Control address |
| DATA     | Target position |
| CHECKSUM | Error detection |

---

# 8. Servo Control Module

서보 제어는 `servoserial.py` 모듈을 사용한다.

UART 초기화

```python
self.ser = serial.Serial("/dev/serial0",115200,timeout=0.001)
```

통신 속도

```
115200 bps
```

---

# 9. Servo Control Functions

## Single Servo Control

```python
Servo_serial_control(id, angle)
```

예시

```python
s.Servo_serial_control(1,2400)
```

---

## Dual Servo Control (Simultaneous)

두 서보를 동시에 제어하는 함수

```python
Servo_serial_double_control(id1,angle1,id2,angle2)
```

예시

```python
s.Servo_serial_double_control(1,1800,2,2400)
```

---

# 10. Servo Range

| Servo ID | Function | Range       |
| -------- | -------- | ----------- |
| 1        | PAN      | 600 ~ 3600  |
| 2        | TILT     | 1300 ~ 4095 |

이 범위를 벗어나면 코드에서 자동 보정된다.

---

# 11. Example Code

```python
from servoserial import ServoSerial

servo = ServoSerial()

PAN = 1
TILT = 2

servo.Servo_serial_double_control(PAN,1800,TILT,2400)
```

동작 흐름

```
Python Script
      ↓
servoserial.py
      ↓
UART (/dev/serial0)
      ↓
Servo Control Board
      ↓
Bus Servo Protocol
      ↓
PAN / TILT Servo
```

---

# 12. Features

* UART 기반 Bus Servo 제어
* PAN / TILT 동시 제어
* Raspberry Pi GPIO UART 사용
* Python 기반 제어

---

# 13. Future Work

다음 기능으로 확장 가능하다.

* VR Head Tracking
* WebSocket 기반 원격 제어
* 실시간 카메라 PTZ 제어
* 모바일 앱 연동

```
VR Device
   ↓
WebSocket
   ↓
Raspberry Pi
   ↓
Servo Control
```

---

# 14. C++ Version (PAN / TILT)

Python 파일은 그대로 두고, 동일 프로토콜을 사용하는 C++ 구현을 추가했다.

추가 파일

```
servoserial_cpp.hpp
servoserial_cpp.cpp
two_servo_cpp.cpp
```

빌드

```bash
g++ -std=c++17 -O2 servoserial_cpp.cpp two_servo_cpp.cpp -o two_servo_cpp
```

실행

```bash
./two_servo_cpp
```
