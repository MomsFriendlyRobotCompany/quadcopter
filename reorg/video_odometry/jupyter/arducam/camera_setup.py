#!/usr/bin/env python
# https://www.arducam.com/product/arducam-120fps-global-shutter-usb-camera-board-1mp-720p-ov9281-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi/
# https://www.arducam.com/docs/uvc-camera-module/introduction-to-arducam-uvc-camera-module/
# https://www.arducam.com/docs/uvc-camera-module/access-camera-demo/how-to-access-arducam-ov9281-global-shutter-uvc-camera-using-external-trigger-snapshot-mode/

import cv2
import time
import sys

if len(sys.argv) != 2:
    print(">> camera_setup.py [camera number]")
    print("    camera_setup.py 0")
    sys.exit(1)

camera = int(sys.argv[1])

cap = cv2.VideoCapture(camera, cv2.CAP_AVFOUNDATION)

# why don't these work?!
cap.set(cv2.CAP_PROP_FPS, 210)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# cap.set(cv2.CAP_PROP_FPS, 210)

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = int(cap.get(5))

print(f">> Camera: {width}x{height} @ {fps}")

cap.release()