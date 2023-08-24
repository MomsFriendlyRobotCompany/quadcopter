
#include <Wire.h>
#include <cstdint>
#include <yivo.hpp>
#include <gciSensors.hpp>
#include <TFmini.h>

#include "imu.hpp"
// #include "pres_temp.hpp"
// #include "motors.hpp"
// #include "blinkled.hpp"

TFmini tfmini;
gciLSOXLISBMP imu; // accel, gyro, mag, press, temp
// BlinkLED blinker(500);
Yivo<256> yivo; // uC to computer packetizer
// Alarm hertz(5);



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

  // ranger
  Serial1.begin(TFmini::DEFAULT_BAUDRATE);
  tfmini.attach(Serial1);

  Serial.println("Boot complete:");
  Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");
  Serial.println("TFmini done");
}

// struct {
//   uint16_t dist;
//   uint16_t str;
//   uint8_t intTime;
// } tf_t;

// tf_t tf;

union {
  uint16_t s[2]; // ushort (uint16 * 2)
  uint8_t  b[5]; // byte (uint8)
} buff;

// tfbuf_t buff;

void loop() {

  if (tfmini.available()) {
    buff.s[0] = tfmini.getDistance();
    buff.s[1] = tfmini.getStrength();
    buff.b[4] = tfmini.getIntegrationTime();
  }

  if (imu.found) {
    imu.read();
  }

  // serial ascii input
  while (Serial.available() > 0) {
    int inByte = Serial.read();

    if (inByte == '\n' or inByte == '\r'); // get rid of \r and \n from Master
    else if (inByte == 't') {
      yivo.pack_n_send(imu.id, imu.data.b, imu.bsize);
      yivo.pack_n_send(22, buff.b, 5); // LidarRanger msg
    }
  }
  
}
