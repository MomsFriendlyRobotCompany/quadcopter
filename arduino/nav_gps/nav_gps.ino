
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <squaternion.hpp>
#include <gciSensors.hpp>
#include <messages.hpp>
#include <yivo.hpp>
#include <quadcopter.hpp>
#include <gecko2.hpp>

#include "motors.hpp"

#define TRUE 1
#define FALSE 0
#define DEBUG FALSE

LSM6DSOX::gciLSM6DSOX sox(&Wire);   // accel and gyro
LIS3MDL::gciLIS3MDL lis3mdl(&Wire); // magnetometer
BMP390::gciBMP390 bmp(&Wire);       // pressure

Adafruit_GPS GPS(&Serial1);
Yivo yivo;
QuadESC esc;
Updated<quad::joystick_t> js;

class Timer {
  public:
  Timer(): epoch(millis()) {}
  void hack() { tmp_epoch = millis(); }
  uint32_t since_hack() { return millis() - tmp_epoch; }
  uint32_t since_epoch() { return millis() - epoch; }

  protected:
  uint32_t epoch;
  uint32_t tmp_epoch;
};

Timer timer;

void setup() {
  Serial.begin(1000000);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_BAUD_115200);

  delay(500);
  GPS.begin(115200);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // $PGTOP,11,value*checksum
  // Value:
  // 1=>Active Antenna Shorted
  // 2=>Using Internal Antenna
  // 3=>Using Active Antenna
  GPS.sendCommand(PGCMD_ANTENNA);
  // GPS.sendCommand(PGCMD_NOANTENNA);

  delay(1000);

  // i2c, fast mode
  Wire.begin();
  Wire.setClock(400000);

  sox.init();
  lis3mdl.init();
  bmp.init();

  // bool setup_done = false;
  while (true) {
    quad::heartbeat_t h;
    YivoPack_t hb = yivo.pack(quad::HEART_BEAT,reinterpret_cast<uint8_t *>(&h),sizeof(h));
    Serial.write(hb.data(), hb.size());

    int avail = Serial.available();
    if (avail <= 6) {
      delay(100);
      continue;
    }

    int num = Serial.available();
    uint8_t id = 0;
    while (num-- > 0) {
      uint8_t b = Serial.read();
      id = yivo.read(b);
      if (id > 0) break;
    }
    
    if (id == quad::CALIBRATION) {
      quad::calibration_t c = yivo.unpack<quad::calibration_t>();

      float params[12];
      memcpy(params, c.a, 12*sizeof(float));
      sox.set_acal(params);
      memcpy(params, c.g, 3*sizeof(float));
      sox.set_gcal(params);
      memcpy(params, c.m, 6*sizeof(float));
      lis3mdl.set_cal(params);

      break;
    }
    else if (id == quad::CONTINUE) {
      quad::continue_t c = yivo.unpack<quad::continue_t>();
      break;
    }
  }
}

void debug_gps() {
  Serial.printf(">> fix: %d sats: %d ant: %d\n",
    GPS.fix, GPS.satellites, GPS.antenna);

  Serial.print(">> pos: ");
  Serial.print(GPS.latitudeDegrees);
  Serial.print("deg, ");
  Serial.print(GPS.longitudeDegrees);
  Serial.print("deg alt: ");
  Serial.print(GPS.altitude);
  Serial.print("m HDOP: ");
  Serial.print(GPS.HDOP);
  Serial.println("m");

  Serial.printf(">> Date: %d-%d-%d Time: %d:%d:%d\n",
    GPS.year, GPS.month, GPS.day,
    GPS.hour, GPS.minute, GPS.seconds);

  Serial.println("------------------------------------");
}

void reboot() {
  NVIC_SystemReset();      // processor software reset
}

void check_serial() {
  if (Serial.available() <= 6) return;

  int num = Serial.available();
  uint8_t id = 0;
  while (num-- > 0) {
    uint8_t b = Serial.read();
    id = yivo.read(b);
    if (id > 0) break;
  }
  
  if (id == quad::CMD_MOTORS) {
    quad::cmd_motors_t m = yivo.unpack<quad::cmd_motors_t>();
    esc.set(m);
  }
  else if (id == quad::REBOOT) reboot();
  // else if (id == MSG_CALIBRATION) {
  //   calibrate_t c = yivo.unpack<calibrate_t>();
  //   float params[12];
  //   memcpy(params, c.params, 12*sizeof(float));
  //   if (c.sensor == calibrate_t::type_t::ACCEL) sox.set_acal(params);
  //   else if (c.sensor == calibrate_t::type_t::GYRO) sox.set_gcal(params);
  //   else if (c.sensor == calibrate_t::type_t::MAG) lis3mdl.set_cal(params);
  // }
}

void loop() {
  // timer.update();

  GPS.read();
  // uint32_t now = millis();
  timer.hack();
  // while ((now - ts) < 10) {
  while (timer.since_hack() < 10) {
    // delay(1);
    GPS.read();
    // now = millis();
  }
  // // float dt = (now - ts) * 0.001;
  // ts = millis();

  if (GPS.newNMEAreceived()) {
    satnav_t gps;
    GPS.parse(GPS.lastNMEA());
    if (GPS.fix) {
      gps.lat = GPS.latitudeDegrees; // decimal degrees
      gps.lon = GPS.longitudeDegrees; // decimal degrees
      gps.altitude = GPS.altitude; // meters above MSL
      gps.satellites = GPS.satellites;
      gps.fix = GPS.fix;
      gps.hdop = GPS.HDOP;

      gps.date.month = GPS.month;
      gps.date.day = GPS.day;
      gps.date.year = GPS.year;
      gps.time.hour = GPS.hour;
      gps.time.minute = GPS.minute;
      gps.time.seconds = GPS.seconds;

#if DEBUG == TRUE
      debug_gps();
#else
      YivoPack_t p = yivo.pack(MSG_SATNAV, reinterpret_cast<uint8_t *>(&gps), sizeof(satnav_t));
      Serial.write(p.data(), p.size());
#endif
    }
  }

  quad::imu_t imu;
  imu.timestamp = timer.since_epoch();
  imu.status = static_cast<uint8_t>(quad::imu_t::IMU_Status::OK);

  LSM6DSOX::sox_t s = sox.read();
  if (s.ok) {
    imu.accel.x = s.ax; // g
    imu.accel.y = s.ay;
    imu.accel.z = s.az;

    imu.gyro.x = s.gx; // rad/s
    imu.gyro.y = s.gy;
    imu.gyro.z = s.gz;
  }
  else imu.status = static_cast<uint8_t>(IMU_Status::A_FAIL) | static_cast<uint8_t>(IMU_Status::G_FAIL);

  LIS3MDL::mag_t mag = lis3mdl.read();
  if (mag.ok) {
    imu.mag.x = mag.x; // uT
    imu.mag.y = mag.y;
    imu.mag.z = mag.z;
  }
  else imu.status |= static_cast<uint8_t>(IMU_Status::M_FAIL);

  BMP390::pt_t pt = bmp.read();
  if (pt.ok) {
    imu.pressure = pt.press; // Pa
    imu.temperature = pt.temp; // C
  }
  else imu.status |= static_cast<uint8_t>(IMU_Status::PT_FAIL);

#if DEBUG == FALSE 
    YivoPack_t p = yivo.pack(quad::IMU, reinterpret_cast<uint8_t *>(&imu), sizeof(quad::imu_t));
    Serial.write(p.data(), p.size());
#endif
}
