#!/usr/bin/env python3
"""
motor_ch8_11_test.py
CH8~CH11 집중 테스트 - 2채널 조합과 3채널 조합을 모두 시도
"""
import time
import smbus

PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
bus = smbus.SMBus(1)

def led_on_l(ch):  return 0x06 + 4 * ch
def led_on_h(ch):  return 0x07 + 4 * ch
def led_off_l(ch): return 0x08 + 4 * ch
def led_off_h(ch): return 0x09 + 4 * ch

def pca9685_init():
    bus.write_byte_data(PCA9685_ADDR, MODE1, 0x00)
    time.sleep(0.005)
    old_mode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    bus.write_byte_data(PCA9685_ADDR, MODE1, (old_mode & 0x7F) | 0x10)
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, 5)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode | 0x80)

def set_pwm(channel, value):
    value = max(0, min(4095, value))
    if value == 0:
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0)
    elif value >= 4095:
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0x10)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), 0)
    else:
        bus.write_byte_data(PCA9685_ADDR, led_on_l(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_on_h(channel), 0)
        bus.write_byte_data(PCA9685_ADDR, led_off_l(channel), value & 0xFF)
        bus.write_byte_data(PCA9685_ADDR, led_off_h(channel), (value >> 8) & 0x0F)

def all_off():
    for ch in range(16):
        set_pwm(ch, 0)

pca9685_init()
print("PCA9685 OK\n")

# ─── 테스트 A: 2채널 방식 (CH_A=PWM, CH_B=0) ───
print("=" * 50)
print("테스트 A: 2채널 방식 (한쪽 PWM, 다른쪽 OFF)")
print("=" * 50)

pairs = [
    (8, 9,   "CH8=PWM CH9=OFF"),
    (9, 8,   "CH9=PWM CH8=OFF"),
    (10, 11, "CH10=PWM CH11=OFF"),
    (11, 10, "CH11=PWM CH10=OFF"),
]

for ch_a, ch_b, desc in pairs:
    input("\n[Enter] -> {}".format(desc))
    all_off()
    set_pwm(ch_a, 3000)
    set_pwm(ch_b, 0)
    print("  가동 중... 바퀴가 돌아가나요? (어느 바퀴? 어느 방향?)")

input("\n[Enter] -> 정지")
all_off()

# ─── 테스트 B: 3채널 방식 (PWM+IN1+IN2) ───
print("\n" + "=" * 50)
print("테스트 B: 3채널 방식 (PWM=속도, IN1=HIGH, IN2=LOW)")
print("=" * 50)

combos = [
    (8,  9,  10, "PWM=CH8  IN1=CH9=H  IN2=CH10=L"),
    (8,  10, 9,  "PWM=CH8  IN1=CH10=H IN2=CH9=L"),
    (9,  8,  10, "PWM=CH9  IN1=CH8=H  IN2=CH10=L"),
    (9,  10, 8,  "PWM=CH9  IN1=CH10=H IN2=CH8=L"),
    (10, 8,  9,  "PWM=CH10 IN1=CH8=H  IN2=CH9=L"),
    (10, 9,  8,  "PWM=CH10 IN1=CH9=H  IN2=CH8=L"),
    (10, 11, 8,  "PWM=CH10 IN1=CH11=H IN2=CH8=L"),
    (11, 10, 8,  "PWM=CH11 IN1=CH10=H IN2=CH8=L"),
    (11, 8,  9,  "PWM=CH11 IN1=CH8=H  IN2=CH9=L"),
    (11, 9,  8,  "PWM=CH11 IN1=CH9=H  IN2=CH8=L"),
]

for pwm, in1, in2, desc in combos:
    input("\n[Enter] -> {}".format(desc))
    all_off()
    set_pwm(in1, 4095)
    set_pwm(in2, 0)
    set_pwm(pwm, 2000)
    print("  가동 중... 바퀴가 돌아가나요?")

input("\n[Enter] -> 정지")
all_off()

# ─── 테스트 C: 풀파워 테스트 ───
print("\n" + "=" * 50)
print("테스트 C: 풀파워 단독 테스트 (각 채널 4095)")
print("=" * 50)

for ch in [8, 9, 10, 11]:
    input("\n[Enter] -> CH{} = 4095 (풀파워, 나머지 OFF)".format(ch))
    all_off()
    set_pwm(ch, 4095)
    print("  CH{} 풀파워! 바퀴가 돌아가나요? 소리만 나나요?".format(ch))

input("\n[Enter] -> 모든 채널 OFF")
all_off()
bus.close()
print("\n테스트 완료!")
print("결과를 알려주세요: 어떤 테스트에서 바퀴가 실제로 돌았는지!")
