import serial

ser = serial.Serial("/dev/serial0", 115200, timeout=0.5)
ser.reset_input_buffer()
ser.write(b"hello")
r = ser.read(10)
print("Received:", r)
ser.close()
