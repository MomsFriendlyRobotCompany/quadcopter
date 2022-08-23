
#include "debug.hpp"
#include "imu.hpp"
#include "motors.hpp"
#include "packer.hpp"
#include <Wire.h>
#include <cstdint>

gciLSOXLIS imu;
gciDPS310 press;
QuadESC motors;

void setup() {

    Serial.begin(115200);
    while (!Serial)
        delay(10);

    // i2c, fast mode
    Wire.begin();
    Wire.setClock(400000);

    // setup sensors
    imu.init();
    press.init();

    Serial.println("Boot complete:");
    Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");
    Serial.println(" " + press.found ? "+ Pressure sensor ready"
                                     : "! Pressure Sensor not found");
}

void loop() {
    if (imu.found) {
        imu.read();
        // printMag();
        // printGyro();
        // printAccel();
        // printQuaternion();
        pack_n_send(imu.id, imu.bsize, imu.data.b);
    }

    if (press.found) {
        press.read();
        pack_n_send(press.id, press.bsize, press.data.b);
    }
}
