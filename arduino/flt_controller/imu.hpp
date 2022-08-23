#pragma once

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD
#include <Wire.h>
#include <cmath>
#include <squaternion.hpp>
#include <stdint.h>

/*
Define IMU message size
*/
constexpr int numfloats = 15;

/*
sensor  F   B   data
----------------------
accel - 3 (12)  0-2
gyro  - 3 (12)  3-5
mag   - 3 (12)  6-8
temp  - 1 (4)   9
q     - 4 (16)  10-13
time  - 1 (4)   14  <- really a unsigned long
-----------------------
total - 15 (60)

timestamp: unsigned long = millis(), overflow in 50 days
*/
class gciLSOXLIS {
  public:
    // static const uint8_t id = uint8_t(Sensors::SOXLIS);

    gciLSOXLIS()
        : found(false), soxFound(false), magFound(false),
          bsize(numfloats * sizeof(float)), id(0xD0),
          invg(1.0 / SENSORS_GRAVITY_STANDARD) {}

    void init() {
        if (sox.begin_I2C()) {
            soxFound = true;

            // Accelerometer ------------------------------------------
            sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            sox.setAccelDataRate(LSM6DS_RATE_208_HZ);

            // Gyros ----------------------------------------------------
            sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
        }

        // Magnetometer -----------------------------------------------------
        if (lis3mdl.begin_I2C()) {
            magFound = true;
            // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
            // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
            // already does this
            lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
            lis3mdl.setDataRate(
                LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
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

            data.f[9] = t.temperature;

            unsigned long now = millis();
            dt = (now - ts) * 0.001;
            ts = now;

            Quaternion w(0, data.f[3], data.f[4], data.f[5]);
            q = q + 0.5 * q * w * dt;

            data.f[10] = q.w;
            data.f[11] = q.x;
            data.f[12] = q.y;
            data.f[13] = q.z;
            data.l[14] = now;
        }

        if (magFound) {
            lis3mdl.getEvent(&mag);

            data.f[6] = mag.magnetic.x;
            data.f[7] = mag.magnetic.y;
            data.f[8] = mag.magnetic.z;
        }
    }

    bool found;

    union {
        byte b[numfloats * sizeof(float)];
        float f[numfloats];
        uint32_t l[numfloats];
    } data;
    const uint8_t bsize; // length of array
    const uint8_t id;

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
    unsigned long ts; // timestamp (msec)
};

/////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_DPS310.h>
#include <cstdint>

constexpr int pressfloats = 2;

class gciDPS310 {
  public:
    // static const uint8_t id = uint8_t(Sensors::DPS310);

    gciDPS310() : bsize(pressfloats * sizeof(float)), found(false) {}

    // Sets up the sensor
    void init() {
        if (dps.begin_I2C()) {
            dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
            dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
            found = true;
        }

        // assume startup is on the ground :P
        // save ground measurements for reference
        read();
        gnd_press = data.f[1];
        gnd_alt = altitude(gnd_press);
    }

    void read() {
        if (found) {
            if (dps.temperatureAvailable() || dps.pressureAvailable()) {
                dps.getEvents(&temp_event, &pressure_event);
                data.f[0] = temp_event.temperature;
                data.f[1] = pressure_event.pressure;
            }
        }
    }

    float altitude(const float p) {
        // Probably best not to run here ... very computational.
        // pre compute some of this?
        // call atmospalt() ... like matlab?
        // same as mean sea level (MSL) altitude
        // Altitude from pressure:
        // https://www.mide.com/air-pressure-at-altitude-calculator
        // const float Tb = 15; // temperature at sea level [C] - doesn't work
        // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
        const float Tb = 288.15;          // temperature at sea level [K]
        const float Lb = -0.0065;         // lapse rate [K/m]
        const float Pb = 101325.0;        // pressure at sea level [Pa]
        const float R = 8.31446261815324; // universal gas const [Nm/(mol K)]
        const float M = 0.0289644;        // molar mass of Earth's air [kg/mol]
        const float g0 = 9.80665;         // gravitational const [m/s^2]

        return (Tb / Lb) * (std::pow(p / Pb, -R * Lb / (g0 * M)) - 1.0);
    }

    bool found;

    union {
        byte b[pressfloats * sizeof(float)];
        float f[pressfloats];
    } data;
    const uint8_t bsize; // length of array

  protected:
    Adafruit_DPS310 dps; // pressure / temperature
    sensors_event_t temp_event, pressure_event;
    float gnd_press;
    float gnd_alt;
};
