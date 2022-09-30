
#include "imu.hpp"
#include "pres_temp.hpp"
#include "motors.hpp"
#include "blinkled.hpp"
#include <Wire.h>
#include <cstdint>
#include <yivo.hpp>
#include <gciSensors.hpp>

gciLSOXLISBMP imu;
QuadESC motors(1000);
BlinkLED blinker(500);
Yivo<256> yivo;
Hertz hertz(5);


void setup() {

    Serial.begin(1000000);
    Serial.setTimeout(5);
    while (!Serial)
        delay(10);

    // i2c, fast mode
    Wire.begin();
    Wire.setClock(400000);

    // setup sensors
    imu.init();

    Serial.println("Boot complete:");
    Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");

    Serial.println(" Motors ready");
    
}

bool telemetry = false;
Buffer<15> dbuff;

void sendTelemetry (bool now=false) {
  if (hertz.check() || now) {
    if (imu.found) {
        imu.read();
        if (telemetry || now) yivo.pack_n_send(imu.id, imu.data.b,imu.bsize);
    }
    // else Serial.println("no imu");
  }

  if (motors.check()) {
    Motors4_t m = motors.get_msg();
    if (telemetry) yivo.pack_n_send(MOTORS, reinterpret_cast<uint8_t*>(&m), 10);
  }
}

void loop() {
  sendTelemetry();

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
        else if (inByte == 'r') motors.ramp(); // remove? Why do this at the embedded level?
        else if (inByte == 's') motors.stop();
        else if (inByte == 't') telemetry = !telemetry;
        else if (inByte == 'T') sendTelemetry(true);
        else yivo.pack_n_send(YIVO_ERROR); // send error, unknown command
    }

    blinker.update();

    // if (telemetry) {
    //   // motors.update();
    // }
}
