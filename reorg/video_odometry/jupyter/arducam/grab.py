#!/usr/bin/env python
# https://www.arducam.com/product/arducam-120fps-global-shutter-usb-camera-board-1mp-720p-ov9281-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi/
# https://www.arducam.com/docs/uvc-camera-module/introduction-to-arducam-uvc-camera-module/
# https://www.arducam.com/docs/uvc-camera-module/access-camera-demo/how-to-access-arducam-ov9281-global-shutter-uvc-camera-using-external-trigger-snapshot-mode/

import cv2
import time

cap = cv2.VideoCapture(1, cv2.CAP_AVFOUNDATION)
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

ok = False
while (ok is False):
    time.sleep(1)
    ok, frame = cap.read()
    print('.', end="", flush=True)

sz = frame.shape
print(f">> Image: {sz[0]}x{sz[1]}")

try:
    while True:
        ok, frame = cap.read()

        if not ok:
            print("** no image **")
            break

        cv2.imshow("win", frame)

        c = cv2.waitKey(10)
        if c == 27:
            break
        elif c == ord('s'):
            cv2.imwrite("image.png", frame)

except KeyboardInterrupt:
    print("ctl-c ...")
    cap.release()
    cv2.destroyAllWindows()
