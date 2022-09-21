#!/usr/bin/env python3
# import struct
from serial import Serial
import pickle
from collections import deque

# imu = struct.Struct("<14fI")
# press = struct.Struct("<2f")

data = {
    "accel": deque(),
    "gyro": deque(),
    "temp": deque(),
    "ts": deque(),
}

def printIMU(msg):
    print("-------------------")
    for m in msg:
        print(f"  {m}")

def printPressure(msg):
    print("-------------------")
    print(f"> {msg[0]:.3f} C   {msg[1]:.3f} Pa")

def saveMsg(msgID, msg):
    if msgID == 0xD0:
        data["imu"].append(msg)
    elif msgIX == 0xD1:
        data["press"].append(msg)

###############################################################
s = Serial()
s.port = "/dev/tty.usbmodem14601"
s.baud = 115200
s.timeout = 1
s.open()

try:
    while True:
        line = s.readline().decode("utf8").strip("\r\n").split('\t')
        # print(line)
        nums = list(map(float, line))

        if len(nums) != 8:
            continue

        data["accel"].append(nums[:3])
        data["gyro"].append(nums[3:6])
        data["temp"].append(nums[6])
        data["ts"].append(nums[7])

except KeyboardInterrupt:
    print("ctrl-c")

finally:
    s.close()

    if len(data["accel"]) > 0:
        filename = "data.pkl"
        with open(filename,"wb") as fd:
            pickle.dump(data, fd)
        print(f">> Saved {len(data['accel'])} data points to {filename}")
