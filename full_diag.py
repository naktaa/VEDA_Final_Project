"""
서보 종합 진단 스크립트
- 모든 보드레이트에서 Ping 시도
- 모든 ID(1~10)에서 Ping 시도
- RX에 아무 데이터라도 오는지 확인
- 쓰기 명령으로 움직임 확인
"""
import serial
import time
import sys

DEV = "/dev/serial0"
BAUDS = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400, 9600]
IDS = list(range(0, 11))  # ID 0~10

def make_ping(sid):
    return bytes([0xFF, 0xFF, sid, 0x02, 0x01, (~(sid + 0x02 + 0x01)) & 0xFF])

def make_write_pos(sid, pos):
    """간단한 위치 쓰기 패킷 (기존 pantilt_test.cpp 방식)"""
    pos_H = (pos >> 8) & 0xFF
    pos_L = pos & 0xFF
    time_H = 0x00
    time_L = 0x64  # 100ms
    length = 0x07
    cmd = 0x03
    addr = 0x2A
    chk = (~(sid + length + cmd + addr + pos_H + pos_L + time_H + time_L)) & 0xFF
    return bytes([0xFF, 0xFF, sid, length, cmd, addr, pos_H, pos_L, time_H, time_L, chk])

print("=" * 50)
print("서보 종합 진단")
print("=" * 50)

# ---- Part 1: 전체 보드레이트 + ID 스캔 ----
print("\n[1] Ping 스캔 (모든 보드레이트 x ID 0~10)")
found = False
for baud in BAUDS:
    try:
        ser = serial.Serial(DEV, baud, timeout=0.05)
    except Exception as e:
        print(f"  BAUD {baud}: 열기 실패 - {e}")
        continue

    hits = []
    for sid in IDS:
        ser.reset_input_buffer()
        pkt = make_ping(sid)
        ser.write(pkt)
        ser.flush()
        time.sleep(0.01)
        resp = ser.read(50)
        if resp:
            hits.append((sid, resp))
            found = True

    if hits:
        print(f"  BAUD {baud}: *** 응답 발견! ***")
        for sid, resp in hits:
            print(f"    ID {sid}: {resp.hex(' ')}")
    else:
        print(f"  BAUD {baud}: 응답 없음")
    ser.close()

if not found:
    print("\n  >> 모든 보드레이트/ID에서 응답 없음")

# ---- Part 2: RX 라인 노이즈 체크 ----
print("\n[2] RX 라인 노이즈 체크 (2초간 수신 대기)")
ser = serial.Serial(DEV, 115200, timeout=2.0)
ser.reset_input_buffer()
noise = ser.read(100)
if noise:
    print(f"  수신된 데이터: {noise.hex(' ')}")
else:
    print("  수신 데이터 없음 (조용함)")
ser.close()

# ---- Part 3: 쓰기 명령으로 움직임 확인 ----
print("\n[3] 위치 쓰기 테스트 (115200, ID 1,2 -> 중앙값)")
print("    서보가 움직이는지 눈으로 확인하세요!")
ser = serial.Serial(DEV, 115200, timeout=0.1)
ser.reset_input_buffer()

# ID 1 -> 2048 (중앙)
pkt1 = make_write_pos(1, 2048)
ser.write(pkt1)
ser.flush()
print(f"  ID 1 TX: {pkt1.hex(' ')}")
time.sleep(0.5)

# ID 2 -> 2048 (중앙)
pkt2 = make_write_pos(2, 2048)
ser.write(pkt2)
ser.flush()
print(f"  ID 2 TX: {pkt2.hex(' ')}")
time.sleep(0.5)

ser.close()

print("\n" + "=" * 50)
if found:
    print("결과: 서보 응답 확인됨! 위 결과 참고하세요.")
else:
    print("결과: 어떤 보드레이트/ID에서도 응답 없음.")
    print("  -> 서보가 죽었거나, 배선/어댑터 보드 문제 가능성 높음")
    print("  -> 서보가 Part 3에서 움직였나요?")
    print("     움직임 O -> 서보 살아있음, 응답 회로만 문제")
    print("     움직임 X -> 서보 사망 또는 어댑터 배선 문제")
print("=" * 50)
