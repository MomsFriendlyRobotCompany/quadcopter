#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import time
# from collections import deque
import matplotlib.animation as animation

fig, axs = plt.subplots(1, 2)

xs = []
ys = []
xs2 = []
ys2 = []

sensor = np.cos(np.arange(0,7,0.01))
sensor2 = np.sin(np.arange(0,7,0.01))

# This function is called periodically from FuncAnimation
def animate(i, xs, ys, xs2, ys2):

    sz = 20

    # Add x and y to lists
    xs.append(i)
    ys.append(sensor[i])
    xs = xs[-sz:]
    ys = ys[-sz:]

    xs2.append(i)
    ys2.append(sensor2[i])
    xs2 = xs2[-sz:]
    ys2 = ys2[-sz:]

    # Draw x and y lists
    axs[0].clear()
    axs[0].plot(xs, ys)
    # axs[0].axis('equal')

    axs[1].clear()
    axs[1].plot(xs2, ys2)

    # print(xs)

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys,xs2, ys2), interval=1)
plt.show()