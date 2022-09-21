#!/usr/bin/env python3
# import struct
from serial import Serial
import pickle
from collections import deque
from yivo import Yivo

# imu = struct.Struct("<14fI")
# press = struct.Struct("<2f")

data = deque()

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
s.baud = 1000000
s.timeout = 1
s.open()

yivo = Yivo()

# s.write(b't') # start data stream

try:
    while True:
        msg = yivo.read_packet(s)
        # print(msg)

        if msg is None:
            print("x",end="",flush=True)
        else:
            print(".",end="",flush=True)
            data.append(msg)
        # c = s.read(1)
        # c = ord(c)
        # while c != 0xFF: # find first 0xFF
        #     c = s.read(1)
        #     c = ord(c)
        # c = s.read(1) # should be second 0xFF
        # c = ord(c)
        # if c != 0xFF: # reset to beginning if no 0xFF
        #     continue

        # msgID, size = s.read(2)
        # if msgID in [0xD0, 0xD1]:
        #     # print(f"> size {size}, msg {msg}")

        #     payload = s.read(size)
        #     if msgID == 0xD0:
        #         msg = imu.unpack(payload)
        #         printIMU(msg)
        #     if msgID == 0xD1:
        #         msg = press.unpack(payload)
        #         printPressure(msg)

        #     # saveMsg(msgID, msg)

except KeyboardInterrupt:
    print("ctrl-c")
except Exception as e:
    print(e)

finally:
    # s.write(b't') # stop data stream
    s.close()

    if len(data) > 0:
        filename = "agmqpt.pkl"
        with open(filename,"wb") as fd:
            pickle.dump(data, fd)
        print(f">> Saved {len(data)} data points to {filename}")
