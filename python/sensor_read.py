#!/usr/bin/env python3
import struct
from serial import Serial

imu = struct.Struct("<14fI")
press = struct.Struct("<2f")

def printIMU(payload):
    msg = imu.unpack(payload)

    print("-------------------")
    for m in msg:
        print(f"  {m}")

def printMag(payload):
    msg = press.unpack(payload)

    print("-------------------")
    print(f"> {msg[0]:.3f} C   {msg[1]:.3f} Pa")

###############################################################
s = Serial()
s.port = "/dev/tty.usbmodem14601"
s.baud = 115200
s.timeout = 1
s.open()

try:
    while True:
        c = s.read(1)
        c = ord(c)
        while c != 0xFF: # find first 0xFF
            c = s.read(1)
            c = ord(c)
        c = s.read(1) # should be second 0xFF
        c = ord(c)
        if c != 0xFF: # reset to beginning if no 0xFF
            continue

        msg, size = s.read(2)
        if msg not in [0xD0]:
            continue

        # print(f"> size {size}, msg {msg}")
        payload = s.read(size)
        printIMU(payload)
except KeyboardInterrupt:
    print("ctrl-c")

finally:
    s.close()
