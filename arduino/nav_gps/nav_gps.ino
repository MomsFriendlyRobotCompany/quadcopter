
#include <Wire.h>
// #include <Adafruit_GPS.h>
#include <gcigps.hpp>
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
gci::GPS gps;
#define gpscomm Serial1

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

  gpscomm.begin(9600);
  gpscomm.print(GCI_BAUD_115200);

  delay(500);

  gpscomm.begin(115200);
  gpscomm.print(GCI_RMCGGA);
  gpscomm.print(GCI_UPDATE_1HZ);
  gpscomm.print(GCI_NOANTENNA);

  delay(500);

  // i2c, fast mode
  Wire.begin();
  Wire.setClock(400000);

  sox.init();
  lis3mdl.init();
  bmp.init();
}

void send_heartbeat() {
  quad::heartbeat_t h;
  YivoPack_t hb = yivo.pack(quad::HEART_BEAT,reinterpret_cast<uint8_t *>(&h),sizeof(h));
  Serial.write(hb.data(), hb.size());
}

void reboot() {
  #if defined(ARDUINO_ITSYBITSY_M4)
    NVIC_SystemReset();      // processor software reset
  #elif defined(RASPBERRYPI_PICO)
    watchdog_enable(1, 1);
    while(1);
  #endif
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
  else if (id == quad::CALIBRATION) {
    quad::calibration_t c = yivo.unpack<quad::calibration_t>();
    float params[12];
    memcpy(params, c.a, 12*sizeof(float));
    sox.set_acal(params);
    memcpy(params, c.g, 3*sizeof(float));
    sox.set_gcal(params);
    memcpy(params, c.m, 6*sizeof(float));
    lis3mdl.set_cal(params);
  }
}

void loop() {
  int c = gpscomm.read();
  bool ok = gps.read(c);
  timer.hack();
  while (timer.since_hack() < 10) {
    for (int i=0; i < 10; ++i) {
      c = gpscomm.read();
      ok = gps.read(c);
      if (ok) break;
    }
  }

  if (ok) {
    satnav_t msg{0};
    gci::GpsID id = gps.get_id();
    if (id == gci::GpsID::GGA) {
      gga_t gga{0};
      ok = gps.get_msg(gga);
      if (!ok) return;
      msg.lat = gga.lat;
      msg.lon = gga.lon;
      msg.altitude = gga.msl;
      msg.satellites = gga.num_sats;
      msg.time.hour = gga.utc.hour;
      msg.time.minute = gga.utc.minute;
      msg.time.seconds = gga.utc.seconds;
      
#if DEBUG == TRUE
      Serial.printf("GGA: %f %f\n", gga.lat, gga.lon);
      Serial.print("GGA: ");
      Serial.print(gga.lat);
      Serial.print(" ");
      Serial.println(gga.lon);
#endif
    }
    else if (id == gci::GpsID::RMC) {
      rmc_t rmc{0};
      ok = gps.get_msg(rmc);
      if (!ok) return;
      msg.lat = rmc.lat;
      msg.lon = rmc.lon;
      msg.time.hour = rmc.utc.hour;
      msg.time.minute = rmc.utc.minute;
      msg.time.seconds = rmc.utc.seconds;
      msg.date.month = rmc.date.month;
      msg.date.day = rmc.date.day;
      msg.date.year = rmc.date.year;
      
#if DEBUG == TRUE
      Serial.printf("RMC: %f %f\n", rmc.lat, rmc.lon);
      Serial.print("RMC: ");
      Serial.print(rmc.lat);
      Serial.print(" ");
      Serial.println(rmc.lon);
#endif
    }

#if DEBUG == FALSE 
    YivoPack_t p = yivo.pack(MSG_SATNAV, reinterpret_cast<uint8_t *>(&msg), sizeof(satnav_t));
    Serial.write(p.data(), p.size());
#endif
  }

  quad::imu_t imu;
  imu.timestamp = timer.since_epoch();
  imu.status = static_cast<uint8_t>(IMU_Status::OK);

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
