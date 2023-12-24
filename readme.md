# Quadcopter

**Under Heavy Development**

## Notebooks

Jupyter notebooks for development

## Cpp

- `include`: common definitions and classes to use between computer and Arduino
- `src`: this is mainly for testing and developemnt since it is all designed to
run on a computer

## Data

This folder containes recordings of sensor data

## Mechanical

Fusion360 design of the quadcopter

## Tools

These are tools to help with debugging or development and written in python

## Docker

Developing ways to use docker on a Raspberry Pi for containerizing flight software

## Dependancies

- `gciSensors`: imu, pressure, temperature sensors using I2C
- `gciGps`: gps using serial port
- `squaternions`: quaternion math
- `gecko2`: misc

## Other Quadcopters

- [Arduino Pico](https://github.com/earlephilhower/arduino-pico)
- [WizFi360-EVB-PICO Quadcopter](https://maker.wiznet.io/ravi_maker/contest/wizfi360-evb-pico-based-smart-phone-controlled-micro-drone-for-stem-education/)

## Other Tools

- Sensors
    - [Adafruit VL53L1X Time of Flight Senosr, 30-4000mm](https://www.adafruit.com/product/3967) for $15, but ONLY works indoors (sun light washes out signal)
    - [Pololu Drivers](https://github.com/pololu/vl53l1x-arduino/tree/master) are much smaller and simplier than Adafruit's drivers
    - Alternative (also cheaper) is to use ultrasonics which would work both indoors and outdoors

- [NOAA WMM data](https://www.ncei.noaa.gov/magnetic-model-survey-page?redirect=wmm-coefficients)

- python:
    - Qt5 Instrument Panel: [pyEfis](https://github.com/makerplane/pyEfis)
    - MIT: [pygeomag](https://github.com/boxpet/pygeomag/tree/main)
        - Format of data: https://www.ngdc.noaa.gov/geomag/WMM/wmmformat.shtml
        ```python
        >>> from pygeomag import GeoMag
        >>> geo_mag = GeoMag()
        >>> result = geo_mag.calculate(glat=47.6205, glon=-122.3493, alt=0, time=2023.75)
        >>> print(result.d)
        15.25942260585284
        ```

- [Pico DShot]((https://github.com/cadouthat/pico-dshot) using PIO
- [ESC Configurator](https://esc-configurator.com)

- [simulator: flightmare](https://flightmare.readthedocs.io/en/latest/getting_started/readme.html)

- [Open source signal analyzer](sigrok.org)
    - [pulseview github](https://github.com/sigrokproject)
    - [pico as a signal analyzer](https://github.com/dotcypress/ula)
    - [uLA](https://forums.raspberrypi.com/viewtopic.php?t=350300)


## Resources

- BSD: [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- BSD: [PX4 Estimation and Control Library (ECL)](https://github.com/PX4/PX4-ECL)
- BSD: [PX4 Matrix](https://github.com/PX4/PX4-Autopilot/tree/4a3d64f1d76856d22323d1061ac6e560efda0a05/src/lib/matrix)
- BSD: [InertialNav](https://github.com/priseborough/InertialNav)
- BSD: [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- BSD: [Pico Examples](https://github.com/raspberrypi/pico-examples)
- GPL: [Ardupilot](https://github.com/ArduPilot/ardupilot)
- GPL: [Crazyflie](https://github.com/bitcraze/crazyflie-firmware)
- GPL: [A Simple and Cool Flight Controller (Pi Pico)](https://github.com/victorhook/asac-fc)
- GPL: [asac-gui (tk)](https://github.com/victorhook/asac-gcs/tree/main)
- github: [python computer vision](https://github.com/timmarkhuff/horizon_detector/tree/main) flying plane
- [Udacity Quadcopter EKF](https://github.com/Ashutosh-Badave/4.EKF_estimator_for_Drone)

- [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node/tree/kinetic)
- [fiducial slam](https://github.com/UbiquityRobotics/fiducials/tree/noetic-devel)

# MIT License

**Copyright (c) 2019 Mom's Friendly Robot Company**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
