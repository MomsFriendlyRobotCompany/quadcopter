#!/usr/bin/env python3

import cv2
import pickle


with open("data.pkl", "rb") as fd:
    data = pickle.load(fd)

print(f">> Loaded {len(data)} data points")
print(f">> Images: {data[0]['image'].shape}")



for d in data:
    cv2.imshow("img", d["image"])

    cv2.waitKey(1)

    print(d["lidar"])
