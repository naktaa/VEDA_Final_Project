import serial
import time

DEV = "/dev/serial0"
BAUD = 115200

def wp(sid, pos):
    pH, pL = (pos >> 8) & 0xFF, pos & 0xFF
    L = 7; c = 3; a = 0x2A; tH = 0; tL = 0x0A
    chk = (~(sid + L + c + a + pH + pL + tH + tL)) & 0xFF
    ser.write(bytes([0xFF, 0xFF, sid, L, c, a, pH, pL, tH, tL, chk]))
    ser.flush()

ser = serial.Serial(DEV, BAUD, timeout=0.1)

print("브로드캐스트 1600 이동")
wp(0xFE, 1600)
time.sleep(2)

print("브로드캐스트 2200 이동")
wp(0xFE, 2200)
time.sleep(2)

print("원위치 1800")
wp(0xFE, 1800)

ser.close()
print("완료 - 팬 서보도 움직였나요?")
