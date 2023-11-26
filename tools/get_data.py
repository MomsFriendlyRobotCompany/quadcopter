#!/usr/bin/env python3
from serial import Serial
from serial.tools.list_ports import comports, grep
from messages import *
from the_collector import Collector
from yivo import Yivo
from yivo import MsgInfo
from yivo import Errors as YErrs
from colorama import Fore
import time
import sys  # command line args and exit

cmdline = ""
if len(sys.argv) == 2:
    cmdline = sys.argv[1]

info = {
    "LSM6DSOX": {
        "accel": {
            "range": (-4,4),
            "units": "g",
            "cal": [[ 1.00268927, -0.00056029, -0.00190925, -0.00492348],
                    [-0.00138898,  0.99580818, -0.00227335,  0.00503835],
                    [-0.01438271,  0.00673172,  0.9998954 , -0.01364759]]
        },
        "gyro": {
            "range": (-2000,2000),
            "units": "deg/sec",
            "cal": [-0.00889949, -0.00235061, -0.00475294]
        }
    },
    "LIS3MDL": {
        "range": (-4,4), # 4 gauss = 400 uT
        "units": "gauss",
        "cal": {
            "bias": [-13.15340002, 29.7714855, 0.0645215],
            "diag": [0.96545537,0.94936676,0.967698]
        }
    },
    "DPS390": {
        "units": ("C", "Pa") # celcius, pascals
    },
    "Ultimate GPS v3": {
        "units": {
            "lat-lon": "DD", # decimal degrees
            "altitude": "m",
            "hdop": "m"
        }
    },
    "AGMPT": "ax ay az gx gy gz mx my mz  temp press timestamp"
}

gps = []
imu = []

class Hertz:
    def __init__(self):
        self.reset()

    def reset(self):
        self.epoch = time.monotonic()
        self.cnt = 0

    def touch(self):
        self.cnt += 1

    def hertz(self):
        ts = time.monotonic()
        return self.cnt/(ts - self.epoch)


def main():
    msgs = [
        imu_t,
        pose_t,
        gps_t
    ]
    msgdb = MsgInfo()
    for msg in msgs:
        fmt,_,cls,msgid = msg.__yivo__(0)
        print(fmt, cls, msgid)
        msgdb[msgid] = (fmt, cls)

    yivo = Yivo(msgdb)

    imuhz = Hertz()
    gpshz = Hertz()

    ports = comports()
    port = None
    for p in ports:
        if "usbmodem" in p.device:
            port = p.device
            break
    print(port)

    # port = "/dev/cu.usbmodem14501"
    # ser = Serial(port, 1000000, timeout=0.001)
    ser = Serial(port, 1000000)
    if not ser.is_open:
        print(f"{Fore.RED}*** Failed to open: {port} ***{Fore.RESET}")
        sys.exit(1)

    try:
        while True:
            this_id = 0
            while this_id == 0:
                c = ser.read(1)
                # print(">> ", c)
                if(len(c) > 0):
                    this_id = yivo.parse(c)

            err, msg = yivo.unpack()
            if err == 0:
                print(msg)
                if this_id == Msg.IMU:
                    imuhz.touch()
                    imu.append(msg.serialize())
                elif this_id == Msg.SATNAV:
                    gpshz.touch()
                    gps.append(msg.serialize())

                # print("\x1Bc\n")
                # print(f">> IMU: {imuhz.hertz():7.1f} Hz", end="\n")
                # print(f">> GPS: {gpshz.hertz():7.1f} Hz", end="\r")
            else:
                print(f"{Fore.RED}*** {Msg.str(this_id)} has error: {YErrs.str(err)} ***{Fore.RESET}")

    except KeyboardInterrupt:
        print("\nctrl-c")
    except Exception as e:
        print(f"\n{e}\nSomething happened ...")
    finally:
        ser.close()

    if 0:
        data = {
            "imu": imu,
            "gps": gps
        }
        print(f">> IMU: {len(imu)} GPS: {len(gps)} data points")
        col = Collector()
        col.timestamp = False
        fname = cmdline
        col.write(fname, data, info)

main()