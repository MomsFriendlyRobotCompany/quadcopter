# Accels and Gyros

- Gyroscopes: good for short duration
- Accelerometers: good for long durations
    - sensative to vibrations

## Complemenatary Filter

Not the best, but a simple method to combine the strengths of the sensors to
determine the angle (roll or pitch, but not heading).

```cpp
constexpr float a = 0.995f; // gyro

float dt; // set time step
float g_angle = gyro * dt; // integrate gyro reading to get angle
float a_angle; // get angle from accel

angle = a *(angle + g_angle) + (1.0f - a) * a_angle;
```