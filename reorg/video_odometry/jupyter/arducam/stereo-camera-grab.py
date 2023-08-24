#!/usr/bin/env python
import sys
sys.path.append('..')

import cv2
from opencv_camera import bgr2gray
import time
from slurm.files import rm
import pickle

from helpers import SaveVideo

frames = []
save_video = False

cap = cv2.VideoCapture(0)
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

# fname = "flow.mp4"
# rm(fname)

ok = False
while (ok is False):
    time.sleep(1)
    ok, frame = cap.read()
    print('.', end="", flush=True)

sz = frame.shape[:2]
print(f">> Image: {sz[0]}x{sz[1]}")

try:
    while True:
        ok, frame = cap.read()

        if not ok:
            print("** no image **")
            break

        frame = cv2.resize(frame, (sz[1]//2, sz[0]//2), interpolation = cv2.INTER_AREA)
        frame = bgr2gray(frame)
        cv2.imshow(f"{frame.shape}", frame)

        c = cv2.waitKey(10)
        if c == 27:
            break
        elif c == ord('g'):
            cv2.imwrite("image.png", frame)
        elif c == ord('s'):
            save_video = not save_video

        if save_video:
            frames.append(frame)

except KeyboardInterrupt:
    print("ctl-c ...")

finally:
    cap.release()
    cv2.destroyAllWindows()

    if len(frames) > 0:
        # vs = SaveVideo()
        # vs.write_list(frames, "flow-stereo.mp4")
        # Video.from_file("flow.mp4")

        with open("flow-stereo.pkl", "wb") as fd:
            pickle.dump(frames, fd)
