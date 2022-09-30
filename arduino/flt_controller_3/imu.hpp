#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <squaternion.hpp>
#include <gciSensors.hpp>
#include <cstdint>
#include "sensor.hpp"

/*
This is a wrapper around a group of sensors running approx
at the same update rate. All data is stored in a buffer
and sent to a computer using Serial.write(buff, leng)
command.

F = float
UL = unsigned long (uint32_t)

sensor     Size   B   data
------------------------------
accel     - 3F  (12)  0-2
gyro      - 3F  (12)  3-5
imu temp  - 1F  (4)   6
mag       - 3F  (12)  7-9
q         - 4F  (16)  10-13
press     - 1F  (4)   14
air temp  - 1F  (4)   15
time      - 1UL (4)   16 
-----------------------------
total     - 17 (xx)  x 100 Hz x 8b = xx00 bps = xx kbps

Overhead: 6B x 8b x 100 Hz = 4,800 bps = 4.8 kbps

Motors: 4F x 4B x 8b x 100 Hz = 12,800 bps = 12.8 kbps

timestamp: unsigned long = millis(), overflow in 50 days
*/
constexpr uint8_t imuNumFloats = 17;

class gciLSOXLISBMP: public mSensor<imuNumFloats> {
  public:

    gciLSOXLISBMP(): mSensor<imuNumFloats>(IMU_AGMQPT),
        soxFound(false), magFound(false), pressFound(false),
        sox(&Wire), lis3mdl(&Wire), bmp(&Wire) {}

    void init() {
        if (sox.init()) {
            soxFound = true;

            // Accelerometer ------------------------------------------
            // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            // sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

            // Gyros ----------------------------------------------------
            // sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            // sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
        }

        // Magnetometer -----------------------------------------------------
        if (lis3mdl.init()) {
            magFound = true;
            // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
            // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
            // already does this
            // lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
            // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
            // lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        }

        if (bmp.init()) {
          pressFound = true;
        }

        found = magFound && soxFound && pressFound;
        ts = millis();
    }

    void read() {
        if (soxFound) {
            gci::sox_t s = sox.read();

            data.f[0] = s.ax;// * invg;
            data.f[1] = s.ay;// * invg;
            data.f[2] = s.az;// * invg;

            data.f[3] = s.gx;
            data.f[4] = s.gy;
            data.f[5] = s.gz;

            data.f[6] = s.temp;

            uint32_t now = millis();
            dt = (now - ts) * 0.001;
            ts = now;

            Quaternion w(0, data.f[3], data.f[4], data.f[5]);
            q = q + 0.5 * q * w * dt;

            data.f[10] = q.w;
            data.f[11] = q.x;
            data.f[12] = q.y;
            data.f[13] = q.z;

            data.l[16] = now; // time
        }

        if (magFound) {
            gci::mag_t s = lis3mdl.read();

            data.f[7] = s.x;
            data.f[8] = s.y;
            data.f[9] = s.z;
        }

        if (pressFound) {
          gci::pt_t s = bmp.read();
          if (s.ok) {
            data.f[14] = s.press;
            data.f[15] = s.temp;
          }
          else {
            data.f[14] = 0.0f;
            data.f[15] = 0.0f;
          }
        }
    }

  protected:
    bool soxFound;
    bool magFound;
    bool pressFound;

    gci::gciLSM6DSOX sox;    // accel and gyro
    gci::gciLIS3MDL lis3mdl; // magnetometer
    gci::gciBMP390 bmp;     // pressure
    Quaternion q;
    float dt;         // time difference between samples
    uint32_t ts; // timestamp (msec)
};












#if 0 /////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD
// #include <Wire.h>
// #include <cmath>
#include <squaternion.hpp>
// #include <stdint.h>
// #include <cstdint>
// #include <yivo.hpp> // MsgIDs
#include "sensor.hpp" // Sensor, Buffer




/*
F = float
UL = unsigned long (uint32_t)

sensor Size   B   data
----------------------
accel - 3F  (12)  0-2
gyro  - 3F  (12)  3-5
mag   - 3F  (12)  6-8
q     - 4F  (16)  9-12
temp  - 1F  (4)   13
time  - 1UL (4)   14 
-----------------------
total - 15 (60)

timestamp: unsigned long = millis(), overflow in 50 days
*/
class gciLSOXLIS: public mSensor<15> {
  public:

    // gciLSOXLIS()
    //     : found(false), soxFound(false), magFound(false),
    //       bsize(numfloats * sizeof(float)), 
    //       id(IMU_AGMQT),
    //       invg(1.0 / SENSORS_GRAVITY_STANDARD) {}

    gciLSOXLIS(): mSensor<15>(IMU_AGMQT),
        soxFound(false), magFound(false),
        invg(1.0 / SENSORS_GRAVITY_STANDARD) {}

    void init() {
        if (sox.begin_I2C()) {
            soxFound = true;

            // Accelerometer ------------------------------------------
            sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            // sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
            sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

            // Gyros ----------------------------------------------------
            sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            // sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
            sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
        }

        // Magnetometer -----------------------------------------------------
        if (lis3mdl.begin_I2C()) {
            magFound = true;
            lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
            // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
            // already does this
            lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
            // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
            lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        }

        found = magFound && soxFound;
        ts = millis();
    }

    void read() {
        if (soxFound) {
            sox.getEvent(&a, &g, &t);

            data.f[0] = a.acceleration.x * invg;
            data.f[1] = a.acceleration.y * invg;
            data.f[2] = a.acceleration.z * invg;

            data.f[3] = g.gyro.x;
            data.f[4] = g.gyro.y;
            data.f[5] = g.gyro.z;

            data.f[13] = t.temperature;

            uint32_t now = millis();
            dt = (now - ts) * 0.001;
            ts = now;

            Quaternion w(0, data.f[3], data.f[4], data.f[5]);
            q = q + 0.5 * q * w * dt;

            data.f[9] = q.w;
            data.f[10] = q.x;
            data.f[11] = q.y;
            data.f[12] = q.z;

            data.l[14] = now; // time
        }

        if (magFound) {
            lis3mdl.getEvent(&mag);

            data.f[6] = mag.magnetic.x;
            data.f[7] = mag.magnetic.y;
            data.f[8] = mag.magnetic.z;
        }
    }

  protected:
    bool soxFound;
    bool magFound;
    Adafruit_LSM6DSOX sox;    // accel and gyro
    Adafruit_LIS3MDL lis3mdl; // magnetometer
    const float invg;
    sensors_event_t a, g, t;
    sensors_event_t mag;
    Quaternion q;
    float dt;         // time difference between samples
    uint32_t ts; // timestamp (msec)
};

#endif
