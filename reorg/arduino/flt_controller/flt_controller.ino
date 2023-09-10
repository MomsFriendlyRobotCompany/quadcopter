
#include "debug.hpp"
#include "imu.hpp"
#include "motors.hpp"
#include "packer.hpp"
#include "blinkled.hpp"
#include <Wire.h>
#include <cstdint>
#include <yivo.hpp>

gciLSOXLIS imu;
gciDPS310 press;
QuadESC motors(1000);
BlinkLED blinker(500);
Yivo yivo;
Hertz hertz(5);


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

    // motors.init();
    // motors.arm();
    Serial.println(" Motors ready");
    
}

bool telemetry = false;

void sendTelemetry () {
  if (hertz.check()) {
    if (imu.found) {
        imu.read();
        // printMag();
        // printGyro();
        // printAccel();
        // printQuaternion();
        yivo.pack_n_send(imu.id, imu.data.b,imu.bsize);
    }

    if (press.found) {
        press.read();
        yivo.pack_n_send(press.id, press.data.b,press.bsize);
    }
  }

  if (motors.check()) {
    Motors4_t m = motors.get_msg();
    yivo.pack_n_send(MOTORS, reinterpret_cast<uint8_t*>(&m), 10);
  }
}

void loop() {
  if (telemetry) sendTelemetry();

    // serial ascii input
    if (Serial.available() > 0) {
        int inByte = Serial.read();

        if (inByte == '\n' or inByte == '\r'); // get rid of \r and \n from Master
        else if (inByte == 'a') motors.arm();
        else if (inByte == 'g') yivo.pack_n_send(PING, nullptr, 0);
        else if (inByte == 'm') {
          Motors4_t m = motors.get_msg();
          yivo.pack_n_send(MOTORS, reinterpret_cast<uint8_t*>(&m), 10);
        }
        else if (inByte == 'p') { // pwm motor command
            // p,m0,m1,m2,m3
            int m[4];
            for (int i=0; i < 4; i++) {
              uint8_t a = Serial.read(); // low byte
              uint8_t b = Serial.read(); // high byte
              m[i] = (b << 8) + a; 
            }
            motors.set(m[0],m[1],m[2],m[3]);
        }
        else if (inByte == 'r') motors.ramp(); // remove ?
        else if (inByte == 's') motors.stop();
        else if (inByte == 't') telemetry = !telemetry;
        else if (inByte == 'T') sendTelemetry();
        else yivo.pack_n_send(YIVO_ERROR); // send error, unknown command
    }

    blinker.update();

    // if (telemetry) {
    //   // motors.update();
    // }
}