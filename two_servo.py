# two_servos_seq.py
from servoserial import ServoSerial
import time

s = ServoSerial()

PAN = 1
TILT = 2

pan_vals  = [600, 1200, 1800, 2400, 3000, 3600, 3000, 2400, 1800, 1200]
tilt_vals = [1300, 1800, 2300, 2800, 3300, 3800, 4095, 3800, 3300, 2800, 2300, 1800]

while True:
    # PAN sweep
    for a in pan_vals:
        s.Servo_serial_control(PAN, a)
        time.sleep(0.25)

    # TILT sweep
    for a in tilt_vals:
        s.Servo_serial_control(TILT, a)
        time.sleep(0.25)
