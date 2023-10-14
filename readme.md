# Quadcopter

## Arduino

- `nav_gps`: some testing with navigation
- `pass_thru`: the uC does nothing but immediately copy sensor data to serial port
- `quadcopter`: this is the real quadcopter software

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

- `gciSensors`: imu, pressure, temperature
- `gciGps`: gps
- `gecko2`

## Resources and Other Projects

- BSD: [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- BSD: [PX4 Estimation and Control Library (ECL)](https://github.com/PX4/PX4-ECL)
- BSD: [PX4 Matrix](https://github.com/PX4/PX4-Autopilot/tree/4a3d64f1d76856d22323d1061ac6e560efda0a05/src/lib/matrix)
- BSD: [InertialNav](https://github.com/priseborough/InertialNav)
- BSD: [Pico SDK](https://github.com/raspberrypi/pico-sdk/tree/master)
- BSD: [Pico Examples]()
- GPL: [Ardupilot](https://github.com/ArduPilot/ardupilot)
- GPL: [Crazyflie](https://github.com/bitcraze/crazyflie-firmware)
- GPL: [A Simple and Cool Flight Controller (Pi Pico)](https://github.com/victorhook/asac-fc)
- GPL: [asac-gui (tk)](https://github.com/victorhook/asac-gcs/tree/main)

- [ESC Configurator](https://esc-configurator.com)

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