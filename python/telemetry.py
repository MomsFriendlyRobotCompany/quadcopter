
import struct
import pickle
from collections import deque

imu = struct.Struct("<14fI")
press = struct.Struct("<2f")
motor = struct.Struct("<5B")


def printIMU(msg):
    print("-------------------")
    for m in msg:
        print(f"  {m}")

def printPressure(msg):
    print("-------------------")
    print(f"> {msg[0]:.3f} C   {msg[1]:.3f} Pa")

class Telemetry:
    data = {
        "imu": deque(),
        "press": deque(),
        "motors": deque()
    }
    save_data = False

    def get(self, s):
        c = s.read(1)

        c = ord(c)
        while c != 0xFF: # find first 0xFF
            c = s.read(1)
            c = ord(c)
        c = s.read(1) # should be second 0xFF
        c = ord(c)
        if c != 0xFF: # reset to beginning if no 0xFF
            return None, None

        msgID, size = s.read(2)
        if msgID in [0xD0, 0xD1, 0xE0]:

            payload = s.read(size)

            if msgID == 0xD0:
                msg = imu.unpack(payload)
                self.handle(msg, "imu")
            elif msgID == 0xD1:
                msg = press.unpack(payload)
                self.handle(msg, "press")
            elif msgID == 0xE0:
                msg = motor.unpack(payload)
                self.handle(msg, "motors")

        else:
            print(f"Bad msg: {hex(msgID)}")

        return msgID, msg

    def handle(self, msg, key):
        if self.save_data is True:
            self.data[key].append(msg)
        else:
            print(msg)

    def save(self):
        if len(self.data["imu"]) > 0 or len(self.data["press"]) > 0:
            filename = "data.pkl"
            with open(filename,"wb") as fd:
                pickle.dump(self.data, fd)
            print(f">> Saved {len(self.data['imu'])} data points to {filename}")