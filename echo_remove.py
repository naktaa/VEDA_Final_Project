# 에코 제거 포함 ping 테스트
import serial
import time
DEV = "/dev/serial0"
BAUD = 115200
ser = serial.Serial(DEV, BAUD, timeout=0.1)
ser.reset_input_buffer()
# Ping 패킷 구성 (ID=1)
sid = 1
packet = bytes([0xFF, 0xFF, sid, 0x02, 0x01, (~(sid + 0x02 + 0x01)) & 0xFF])
print(f"TX ({len(packet)} bytes): {packet.hex(' ')}")
ser.write(packet)
ser.flush()
# TX 완료 대기
time.sleep(0.01)
# 에코 제거: 보낸 만큼 읽어서 버리기
echo = ser.read(len(packet))
print(f"Echo ({len(echo)} bytes): {echo.hex(' ') if echo else 'NONE'}")
# 이제 진짜 응답 읽기
time.sleep(0.01)
resp = ser.read(20)
print(f"Response ({len(resp)} bytes): {resp.hex(' ') if resp else 'NONE'}")
ser.close()