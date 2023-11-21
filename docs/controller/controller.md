# Flight Controller

| Note: I start my motor numbering from 0 instead of from 1.

## Motor Mixing

Given the commands throttle and roll, pitch, and yaw; you can combine these into
the individual motor commands.

```
m0 = throttle - roll - pitch - yaw
m1 = throttle + roll + pitch - yaw
m2 = throttle - roll + pitch + yaw
m3 = throttle + roll - pitch + yaw
```

```
motor = A * v
v = [throttle roll pitch yaw]
A = [
    1 -1 -1 -1,
    1  1  1 -1,
    1 -1  1  1,
    1  1 -1  1
]
```

- ref: [Embedded Programming for Quadcopters](https://www.youtube.com/watch?v=CHSYgLfhwUo)


## Alternate

Assuming perfect `x` on the quad.

CW     CCW
m3     m1
 \  ^  /
  \_|_/
  /   \
 /     \
m2     m0
CCW    CW

```
[m0 m1 m2 m3] = A * v
v = [throttle roll pitch yaw]
A = [
    1 -1  1 -1,
    1 -1 -1  1,
    1  1  1  1,
    1  1 -1 -1
]
```

However, if the `x` is perfectly all 90 degree angles, then you will neex to
recalculate the `A` matrix above. Use the Oscalian link below for an example

- ref: [asac-fc](https://github.com/victorhook/asac-fc/blob/main/docs/index.md)
- ref: Oscarliang [Cleanflight custom motor mixing](https://oscarliang.com/custom-motor-output-mix-quadcopter/)