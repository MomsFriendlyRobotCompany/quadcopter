# Quadcopter

- `dev`: testing out ideas and checking individual things work
- `src`: this is the main quadcopter code
    - `picolib`: eventually move to separate library, but turn `pico-sdk` C into C++
- `c_library_v2`: download `mavlink` version 2 and put it in this folder. **Note** the `.gitignore` doesn't track it, so it is initially **missing**. You might have to rename it

## Timers

| Sensor   | Readings             | Sample Rate Hz  | Timer Hz | Units |
|----------|----------------------|-----------------|----------|-------|
| LSM6DSOX | accels/gyros         | 104             | 100      | g/dps
| LIS3MDL  | mags                 | 154             | 100      | unitless
| BMP390   | pressure/temperature | ?               | ?        | Pa/C
| GPS      | lat/lon/alt          | 1               | 1        | deg/deg/m
| ADC      | Battery              | 1               | 1        | mV

| Function      | States           | Hz | Units |
|---------------|------------------|----|-------|
| Kalman Filter | pos/vel/attitude | 10 | m/mps/unitless
| Message Pub   | all              | 10 | n/a

## ToDo

- [ ] setup for multi-core
    - Need to figure out an architecture that leverages 1 or 2 cores
- [x] investigate Nuttx and FreeRTOS: they seem overly complex
    - I think I can achieve a "good enough" solution using my `Trigger` class and timers
- [ ] figure out mavlink/yivo switching, not sure of value of mavlink yet, because I can't get it working with QGroundControl yet ... it never connects to uav, some sort of handshake error
- [ ] setup some sort of debugging GUI (curses or Tk)
- [x] investigate fixed point math ... a lot of work, but need to see if I need it
    - Might be useful in `gciSensors` FP.16.16 might be useful since the ranges are small
    - Not sure it makes sense for KF, complemantary filter, etc
    - Maybe `Quaternions` since they are [-1,+1] (FP.8.24?)
- [ ] setup some basic filters:
    - Complementary
    - MARG
    - UKF/EKF
- [ ] Need to build up a database of raw data for testing
- [ ] Need to get `Serovs` working

