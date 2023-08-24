#!/usr/bin/env python
# https://www.arducam.com/product/arducam-120fps-global-shutter-usb-camera-board-1mp-720p-ov9281-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi/
# https://www.arducam.com/docs/uvc-camera-module/introduction-to-arducam-uvc-camera-module/
# https://www.arducam.com/docs/uvc-camera-module/access-camera-demo/how-to-access-arducam-ov9281-global-shutter-uvc-camera-using-external-trigger-snapshot-mode/

import cv2
import time
from serial import Serial
from yivo import Yivo, Errors
import sys
from opencv_camera import bgr2gray

from collections import deque
import pickle

save_data = deque()

def get_data(bits):
    data = ser.read(bits)
    if len(data) != bits:
        print(f"** Error {bits} != {len(data)} **")
        return False, None

    err, msgid, data = yivo.unpack(data)
    if err != Errors.NONE:
        print("** Error **")
        return False, None

    return True, data

yivo = Yivo()

ser = Serial(port="/dev/tty.usbmodem14401", baudrate=1000000, timeout=0.01)
if not ser.is_open:
    print("** serial port couldn't be opened **")
    sys.exit(1)

cap = cv2.VideoCapture(1)
# ok, frame = cap.read()
cap.set(cv2.CAP_PROP_FPS, 210)
# wv = cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
# hv = cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
# print(f">> Set width[{wv}]  height[{hv}]")

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = int(cap.get(5))
print(f">> Camera: {width}x{height} @ {fps}")

# why don't these work?!
# cap.set(cv2.CAP_PROP_FPS, 210)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# cap.set(cv2.CAP_PROP_FPS, 210)

ok = False
while (ok is False):
    time.sleep(1)
    ok, frame = cap.read()
    print('.', end="", flush=True)

sz = frame.shape
print(f">> Image: {sz[0]}x{sz[1]}")

count = 0
epoch = time.time()

try:
    while True:
        ok, frame = cap.read()

        if not ok:
            print("** no image **")
            break

        frame = bgr2gray(frame)

        ser.reset_input_buffer()
        ser.write(b't\r')
        time.sleep(0.001)

        # IMU: 74B
        # Lidar: 11B
        ok, imu = get_data(74)
        # print(data)
        # time.sleep(0.003)
        if not ok:
            continue
        ok, lidar = get_data(11)
        if not ok:
            continue

        # print(data)

        """
        https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#ga5bb5a1fea74ea38e1a5445ca803ff121
        cv2.INTER_AREA: resampling using pixel area relation. It may be a preferred method for image decimation, as it gives moire'-free results.
        cv2.INTER_CUBIC: This is slow but more efficient.
        cv2.INTER_LINEAR: This is primarily used when zooming is required.
        """
        small = cv2.resize(frame, (320,240), interpolation = cv2.INTER_AREA)
        # small = frame
        # cv2.imshow("win", small)

        # c = cv2.waitKey(1)
        # if c == 27:
        #     break
        # elif c == ord('s'):
        #     cv2.imwrite("image.png", frame)

        save_data.append(
            {
                "image": small,
                "imu": imu,
                "lidar": lidar,
                "ts": time.time()
            }
        )

        now = time.time()
        print(f">> {count}     {count / (now-epoch)}", end="\r")
        count += 1

except KeyboardInterrupt:
    print("ctl-c ...")
    cap.release()
    cv2.destroyAllWindows()

finally:
    with open("data.pkl", "wb") as fd:
        pickle.dump(save_data, fd)
