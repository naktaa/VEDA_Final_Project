# ping_dual.py
import sys
sys.path.append("./scservo_sdk")
from scservo_sdk import PortHandler, sms_sts, scscl

DEV = "/dev/serial0"
BAUD = 115200
IDS = [1, 2]  # 필요하면 범위 늘리기

port = PortHandler(DEV)
if not port.openPort():
    print("open fail"); exit(1)
if not port.setBaudRate(BAUD):
    print("baud fail"); exit(1)

for proto_name, proto in [("sms_sts", sms_sts), ("scscl", scscl)]:
    handler = proto(port)
    print(f"\n== {proto_name} ==")
    for sid in IDS:
        model, res, err = handler.ping(sid)
        if res == 0 and err == 0:
            print(f"ID {sid} OK, model={model}")
        else:
            print(f"ID {sid} no response (res={res}, err={err})")

port.closePort()
