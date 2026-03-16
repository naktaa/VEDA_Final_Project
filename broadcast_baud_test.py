"""
브로드캐스트 + 멀티 보드레이트 테스트
모든 보드레이트에서 여러 각도로 브로드캐스트 쓰기
"""
import serial
import time

DEV = "/dev/serial0"
BAUDS = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400, 9600]
POSITIONS = [1500, 2000, 2500, 1800]  # 여러 위치값

def wp(ser, sid, pos):
    pH, pL = (pos >> 8) & 0xFF, pos & 0xFF
    L = 7; c = 3; a = 0x2A; tH = 0; tL = 0x0A
    chk = (~(sid + L + c + a + pH + pL + tH + tL)) & 0xFF
    ser.write(bytes([0xFF, 0xFF, sid, L, c, a, pH, pL, tH, tL, chk]))
    ser.flush()

print("=" * 50)
print("브로드캐스트 멀티 보드레이트 테스트")
print("각 보드레이트에서 여러 위치로 이동합니다")
print("팬 서보가 움직이는 순간이 있으면 Ctrl+C 누르세요!")
print("=" * 50)

for baud in BAUDS:
    try:
        ser = serial.Serial(DEV, baud, timeout=0.1)
    except Exception as e:
        print(f"BAUD {baud}: 열기 실패 - {e}")
        continue

    print(f"\n--- BAUD {baud} ---")
    for pos in POSITIONS:
        wp(ser, 0xFE, pos)
        print(f"  위치 {pos} 전송 - 움직임 확인...")
        time.sleep(1.5)

    # 원위치
    wp(ser, 0xFE, 1800)
    time.sleep(0.5)
    ser.close()

print("\n" + "=" * 50)
print("완료! 팬 서보가 어떤 보드레이트에서 움직였나요?")
print("아무데서도 안 움직였으면 서보 사망 확정입니다.")
print("=" * 50)
