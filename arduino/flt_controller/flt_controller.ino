
#include <cstdint>
#include <Wire.h>
#include "imu.hpp"
#include "motors.hpp"
#include "debug.hpp"

/*
Msg  Sensor  Size[F/B]
-------------------
0xD0 IMU      15/60
0xD1 usonic    4/16
0xD2 battery   1/4
*/
constexpr int maxMsgSize = 15*sizeof(float) + 5;
union { byte b[maxMsgSize*sizeof(float)]; float f[maxMsgSize]; unsigned long l[maxMsgSize]; } msgBuf;
void pack_n_send(uint8_t msg, uint8_t size, byte *buff){
  msgBuf.b[0] = 0xFF;
  msgBuf.b[1] = 0xFF;
  msgBuf.b[2] = msg;
  msgBuf.b[3] = size;
  memcpy(&msgBuf.b[4], buff, size);

  Serial.write(msgBuf.b, size + 4);
}

/*
message format

[0xff,0xff, msg, size, ... , time]

messages:
- imu,   100hz, 56B,  44800bps, 
- motors, 10hz,  4B,    320bps, 4 bytes, 0-180 deg, 0.7 deg
- battery, 1hz,  4B,     32bps, 1 int
- usonic, 10hz, 16B,   1280bps, 4 ints
*/


gciLSOXLIS imu;
gciDPS310 press;
QuadESC motors;

void setup() {

  Serial.begin(115200);
  while(!Serial) delay(10);
  
  // i2c, fast mode
  Wire.begin();
  Wire.setClock(400000);

  // setup sensors
  imu.init();
  press.init();

  Serial.println("Boot complete:");
  Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");
  Serial.println(" " + press.found ? "+ Pressure sensor ready" : "! Pressure Sensor not found");
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
  // else Serial.println("IMU error");
  
  // delay(500);
}
