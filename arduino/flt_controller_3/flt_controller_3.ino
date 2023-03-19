
#include "imu.hpp"
#include "sensor.hpp"
#include "motors.hpp"
#include "blinkled.hpp"
#include "toggle.hpp"
#include "heartbeat.hpp"
#include <Wire.h>
#include <cstdint>
#include <yivo.hpp>
#include <gciSensors.hpp>
#include <TFmini.h>
#include <Adafruit_NeoPixel.h>
#include "board.h"



Heartbeat heartbeat(1000);
gciLSOXLISBMP imu; // accel, gyro, mag, press, temp
QuadESC motors(1000);
BlinkLED blinker(500);
Yivo<128> yivo; // uC to computer packetizer
Alarm hertz(5000);

constexpr int LED_PIN = 18;
constexpr int LED_COUNT = 1; // How many NeoPixels are attached to the Arduino?
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

TFmini tfmini;
// uint8_t tfbuf[12]; // reinterpret_cast<uint8_t*>(&m)
// Buffer<3> tfbuf;
// Earth::WGS84_t wgs;


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

    Serial1.begin(TFmini::DEFAULT_BAUDRATE);
    tfmini.attach(Serial1);

    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();

    Serial.println("Boot complete:");
    Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");

    Serial.println(" Motors ready");
    delay(1000);

    // Axis_t a{0.,0.,0.};
    // a.x = 1.0f;
    // a.v[1] = 2.0f;
}

Toggle telemetry;
// bool telemetry = false;
// Buffer<15> dbuff;

void sendTelemetry (bool now=false) {
  if (!hertz.check() && !now) return;
  if (imu.found) {
      imu.read();
      if (telemetry || now) yivo.pack_n_send(imu.id, imu.data.b, imu.bsize);
  }
  if (tfmini.available()) {
    Distance_t msg;
    msg.min_distance = 2; // cm
    msg.max_distance = 1200; // cmm
    msg.id = 0;
    msg.type = Distance_t::LIDAR;
    msg.distance = tfmini.getDistance();
    msg.timestamp = millis();
    // memcpy(tfbuf, &msg, sizeof(Distance_t));
    // if (telemetry || now) yivo.pack_n_send(RANGE, tfbuf, sizeof(Distance_t));
    if (telemetry || now) yivo.pack_n_send(RANGE, reinterpret_cast<uint8_t*>(&msg), sizeof(Distance_t));
    // tfmini.getStrength();
    // tfmini.getIntegrationTime();
  }

  if (heartbeat.check()){
    Heartbeat_t msg = heartbeat.get_msg();
    if (telemetry) yivo.pack_n_send(HEARTBEAT, reinterpret_cast<uint8_t*>(&msg), sizeof(Heartbeat_t));
  }

  if (motors.check()) {
    Motors4_t m = motors.get_msg();
    if (telemetry) yivo.pack_n_send(MOTORS, reinterpret_cast<uint8_t*>(&m), 10);
  }
}

void loop() {
  pixels.setPixelColor(0, pixels.Color(0, 0, 100));
  pixels.show();

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
        else if (inByte == 't') telemetry.toggle(); //telemetry = !telemetry;
        else if (inByte == 'T') sendTelemetry(true);
        else yivo.pack_n_send(YIVO_ERROR); // send error, unknown command
    }

    blinker.update();

    // if (telemetry) {
    //   // motors.update();
    // }
}
