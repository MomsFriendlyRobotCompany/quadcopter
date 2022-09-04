
#include "debug.hpp"
#include "imu.hpp"
#include "motors.hpp"
#include "packer.hpp"
#include "blinkled.hpp"
#include <Wire.h>
#include <cstdint>

gciLSOXLIS imu;
gciDPS310 press;
QuadESC motors(100);
BlinkLED blinker(500);
// Telemetry tel(100);


void setup() {

    Serial.begin(115200);
    // while (!Serial)
    //     delay(10);

    // i2c, fast mode
    Wire.begin();
    Wire.setClock(400000);

    // setup sensors
    // imu.init();
    // press.init(); 

    Serial.println("Boot complete:");
    // Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");
    // Serial.println(" " + press.found ? "+ Pressure sensor ready"
    //                                  : "! Pressure Sensor not found");

    motors.init();
    // motors.arm();
    Serial.println(" Motors ready");
    
}

bool telemetry = false;

void loop() {
  if (telemetry) {
    // if (imu.found) {
    //     imu.read();
    //     // printMag();
    //     // printGyro();
    //     // printAccel();
    //     // printQuaternion();
    //     pack_n_send(imu.id, imu.bsize, imu.data.b);
    // }

    // if (press.found) {
    //     press.read();
    //     pack_n_send(press.id, press.bsize, press.data.b);
    // }
  }

    // serial ascii input
    if (Serial.available() > 0) {
        int inByte = Serial.read();

        if (inByte == '\n' or inByte == '\r'); // get rid of \r and \n from Master
        else if (inByte == 's') motors.stop();
        else if (inByte == 'a') motors.arm();
        else if (inByte == 'w') motors.incr(100);
        else if (inByte == 'x') motors.incr(-100);
        else if (inByte == 'p') { // pwm
            // p,m0,m1,m2,m3
            // since these are 1B (0-255), multiply by 4 (shift by 2) to get 0 - 1020
            int a = (unsigned int)(Serial.read()) << 2;
            int b = (unsigned int)(Serial.read()) << 2;
            int c = (unsigned int)(Serial.read()) << 2;
            int d = (unsigned int)(Serial.read()) << 2;
            motors.set(a,b,c,d);
        }
        else if (inByte == 'r') motors.ramp();
        else if (inByte == 't') telemetry = !telemetry;
        else if (inByte == 'g') Serial.println(">>> Good <<<");
        else { // send error, unknown command
          Serial.println("e");
        }
    }

    blinker.update();

    if (telemetry) {
      motors.update();
    }
}
