#!/usr/bin/env python3
import sys
from time import sleep
from serial import Serial
from yivo import Yivo
from squaternion import Quaternion
from yivo_msgs import IMU, MSG
from yivo_msgs import unpack
from yivo_parser import YivoParser
import pickle

data = {
    "imu": [],
    # "range": []
}

def main(save, port):
    ser = Serial()
    ser.port = port
    ser.baud = 1000000
    ser.timeout = 0.1
    ser.open()

    yivo = Yivo()
    yp = YivoParser(yivo.header)

    try:
        while True:
            c = ser.read()
            if yp.parse(c):
                d, msgid = yp.get_info()
                msg = unpack(d, msgid)
                if msg is None:
                    continue
                # if msgid == MSG.DISTANCE:
                #     data["range"].append(msg)
                if msgid == MSG.IMU_FULL:
                    data["imu"].append(msg)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

        if save:
            with open("data.pkl", "wb") as fd:
                pickle.dump(data, fd)
            print("Saved data file")

if len(sys.argv) < 2:
    print("Usage: app.py port")
    sys.exit(1)

port = sys.argv[1]
save = True
main(save, port)