#!/usr/bin/env python3

from dronesim.sensors import SensorNoise
from dronesim.drone import DroneSim
import time
import numpy as np


def main():
    drone = DroneSim()
    # imu = SensorNoise(0.01, 0.015, 3)
    while 1:
        try:
            # time.sleep(0.1)
            # a = np.array([0,0,1])
            # print(imu.read(a))
            drone.loop_once()
        except KeyboardInterrupt:
            break
        # except Exception as e:
        #     print("Unknown error:", e)

main()