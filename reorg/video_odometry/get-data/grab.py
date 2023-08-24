#!/usr/bin/env python3

'''
--------------------------------------------------
Image: 720x1280
Markers.checkerboard: (7, 10)
Intrinsic Camera Parameters
--------------------------------------------------
 [Camera 1]
  f(x,y): 1129.4 1127.4 px
  principlePoint(x,y): 640.8 357.0 px
  distCoeffs [ 1.39107250e-01 -5.25456587e-01 -1.30556613e-04  9.07833455e-04
  9.33127229e-01]
 [Camera 2]
  f(x,y): 1128.5 1127.2 px
  principlePoint(x,y): 652.1 354.6 px
  distCoeffs [ 1.28455592e-01 -3.69658433e-01  5.09480888e-04 -2.52525323e-04
  4.11286905e-01]
--------------------------------------------------
Extrinsic Camera Parameters
--------------------------------------------------
  R [[ 0.99999112 -0.00148358 -0.00394393]
 [ 0.0014944   0.99999512  0.00274168]
 [ 0.00393984 -0.00274755  0.99998846]]
  T[meter] [[-0.03134446]
 [-0.00025078]
 [-0.00159318]]
  E [[ 1.39283028e-06  1.59385949e-03 -2.46405370e-04]
 [-1.46967190e-03 -8.37569159e-05  3.13503789e-02]
 [ 2.03932801e-04 -3.13446763e-02 -8.69255963e-05]]
  F [[ 4.60170665e-08  5.27552785e-05 -2.80562760e-02]
 [-4.86110558e-05 -2.77543141e-06  1.20330732e+00]
 [ 2.48110107e-02 -1.20419456e+00  1.00000000e+00]]
'''
import numpy as np
import cv2
import time
from pathlib import Path

cap = cv2.VideoCapture(0) #, cv2.CAP_AVFOUNDATION)
# ok, frame = cap.read()

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = int(cap.get(5))
print(f">> Camera: {width}x{height} @ {fps}")

# why don't these work?!
# cap.set(cv2.CAP_PROP_FPS, 210)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# cap.set(cv2.CAP_PROP_FPS, 210)

from collections import deque
from slurm.files import rmdir, mkdir
import yaml

# class Base:


class StereoData:
    camera = deque()
    imu = deque()
    epoch = time.monotonic_ns()
    Kl = None
    Pl = None #np.zeros((3,4))
    dl = None
    Kr = None
    Pr = None #np.zeros((3,4))
    dr = None
    # def __init__(self):
    #     self.camera = deque()
    #     self.imu = deque()
    #     self.epoch = time.monotonic_ns()

    def pushImage(self, img):
        self.camera.append((img,time.monotonic_ns()))

    def pushImu(self, imu):
        self.imu.append(imu) # timestamp in imu message

    def write(self, path):
        if len(self.camera) > 0:
            self._writeCamera(path)
        # if len(self.imu) > 0:
        #     self._writeImu(path)
        if self.Pl is not None and self.Pr is not None:
            self._writeCalib(path)

    def _writeCamera(self, path):
        fullpath = Path(path + "/images")
        # mkdir(fullpath)
        fullpath.mkdir(parents=True,exist_ok=True)

        epoch = self.camera[0][1]

        for im, ts in self.camera:
            # fname = str(ts - epoch) + ".png"
            p = fullpath.joinpath(str(ts - epoch) + ".png")
            cv2.imwrite(str(p), im)

    def _writeImu(self, path):
        fullpath = path + "/imu"
        mkdir(fullpath)

        # for msg in self.imu:
        with open(fullpath + "/imu.yml", "w") as fd:
            yaml.dump(self.imu, fd)

    def _writeCalib(self, path):
        try:
            cameraInfo = {
                "K_left": self.Kl.tolist(),
                "P_left": self.Pl.tolist(),
                "distCoeff_left": self.dl.tolist(),
                "K_right": self.Kr.tolist(),
                "P_right": self.Pr.tolist(),
                "distCoeff_right": self.dr.tolist()
            }
        except:
            return

        fullpath = path + "/calib"
        mkdir(fullpath)

        # for msg in self.imu:
        with open(fullpath + "/calib.yml", "w") as fd:
            yaml.dump(cameraInfo, fd)

    def image(self):
        for im in self.camera:
            yield im

    # def imu(self):
    #     for msg in self.imu:
    #         yield msg

    # @property
    # def p_left(self):
    #     return self.pl

    # @p_left.setter
    # def p_left(self, p):
    #     if not isinstance(p,np.ndarray) or p.shape != (3,4):
    #         raise ValueError(f"P must be numpy array shape (3,4), not {type(p)} {p.shape}")
    #     self.pl = p

    # @property
    # def p_right(self):
    #     return self.pr

    # @p_right.setter
    # def p_right(self, p):
    #     if not isinstance(p,np.ndarray) or p.shape != (3,4):
    #         raise ValueError(f"P must be numpy array shape (3,4), not {type(p)} {p.shape}")
    #     self.pr = p
    def setLeftCamera(self, k, p, d):
        self.Kl = k
        self.Pl = p
        self.dl = d

    def setRightCamera(self, k, p, d):
        self.Kr = k
        self.Pr = p
        self.dr = d



ok = False
while (ok is False):
    time.sleep(1)
    ok, frame = cap.read()
    print('.', end="", flush=True)

sz = frame.shape
print(f">> Image: {sz[0]}x{sz[1]}")

record = False

sd = StereoData()
# sd.Kl = np.array([
#     [1129.4, 0, 640.8],
#     [0, 1127.4, 357.0],
#     [0, 0, 1.0]
# ])
# sd.Pl = np.eye(3,4)
# sd.dl = np.array([1,1,1])
# sd.Kr = np.eye(3)
# sd.Pr = np.eye(3,4)
# sd.dr = np.array([1,1,1])

count = 0

try:
    while True:
        ok, frame = cap.read()

        if not ok:
            print("** no image **")
            break

        frame = cv2.rotate(frame, cv2.ROTATE_180)
        cv2.imshow("win", frame)

        if record:
            sd.pushImage(frame)

        c = cv2.waitKey(10)
        if c == 27:
            break
        elif c == ord('s'):
            cv2.imwrite("./save/" + str(count) + ".png", frame)
            count += 1
        elif c == ord('r'):
            # cv2.imwrite("image.png", frame)
            record = not record
            print(f">> Recording data: {record}")

except KeyboardInterrupt:
    print("\nctl-c ...")

finally:
    sd.write("./data")
    cap.release()
    cv2.destroyAllWindows()
