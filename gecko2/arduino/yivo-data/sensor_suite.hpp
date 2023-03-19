#pragma once


#include <gciSensors.hpp>
#include <yivo.hpp>
// #include <gecko2/sensor.hpp>
#include <gecko2/messages.hpp>
#include <squaternion.hpp>

constexpr uint32_t IMU_DELTA = ALARM_100_HZ;
constexpr uint32_t TFM_DELTA = ALARM_5_HZ;
constexpr uint32_t PRES_DELTA = ALARM_25_HZ;

class SensorSuite {
  public:
  SensorSuite (): sox(&Wire), lis(&Wire), bmp(&Wire), qcf(0.02) {}
      //yivo(nullptr), serial(nullptr) {}

  bool init() {
    // tfmini.attach(Serial1);
    // tf.min_distance = 2;
    // tf.max_distance = 1200;
    // tf.type = distance_t::LIDAR;
    epoch = millis();
    imu_alarm = epoch + IMU_DELTA;
    pres_alarm = epoch + PRES_DELTA;
    // tf_alarm = epoch + TFM_DELTA;

    bool ok = true;

    ok &= sox.init();
    ok &= lis.init();
    ok &= bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X);
    // bmp.setOsMode(BMP390::OS_MODE_PRES_16X_TEMP_2X);
    // bmp.setPowerMode(BMP390::MODE_NORMAL); // continuous sampling
    for (int i; i< 10; ++i) {
      bmp.read();
      delay(40); // 25 Hz
    }

    imu.lidar = 0.0f;

    return ok;
  }

  bool check() {
    bool ret = false;
    // serial->println("check");
    // return;

    // if (yivo == nullptr || serial == nullptr) return;
    // serial->println("good");
    // serial->print(millis());
    // serial->print(" ");
    // serial->println(imu_alarm);

    uint32_t now = millis();
    if (now > imu_alarm) {
      LSM6DSOX::sox_t s = sox.read();
      LIS3MDL::mag_t l = lis.read();
      // BMP390::pt_t b = bmp.read();

      imu.accelleration.x = s.ax;
      imu.accelleration.y = s.ay;
      imu.accelleration.z = s.az;
      imu.gyroscope.x = s.gx;
      imu.gyroscope.y = s.gy;
      imu.gyroscope.z = s.gz;
      imu.magnetometer.x = l.x;
      imu.magnetometer.y = l.y;
      imu.magnetometer.z = l.z;
      // imu.temperature = b.temp;
      // imu.pressure = b.press;
      // imu.altitude = bmp.altitude(b.press);
      imu.timestamp = now;

      // #if 1
      float dt = 0.001*(now - epoch);
      epoch = now;

      Quaternion q = qcf.update(
        s.ax, s.ay, s.az,
        s.gx, s.gy, s.gz,
        dt);
  
      imu.orientation.w = q.w; // FIXME
      imu.orientation.x = q.x;
      imu.orientation.y = q.y;
      imu.orientation.z = q.z;
      // #else
      // imu.orientation.w = 0.0f; // FIXME
      // imu.orientation.x = 0.0f;
      // imu.orientation.y = 0.0f;
      // imu.orientation.z = 0.0f;
      // #endif

      // yivo->pack(MSG_IMU_FULL, reinterpret_cast<uint8_t *>(&imu), sizeof(imu_full_t));
      // serial->write(yivo->get_buffer(), yivo->get_total_size());

      // serial->print(s.az);
      // serial->print(" ");
      // serial->println(imu.accelleration.z);

      imu_alarm = now + IMU_DELTA;
      ret = true;
    }
    if (now > pres_alarm) {
      BMP390::pt_t b = bmp.read();
      imu.altitude = bmp.altitude(b.press);
      pres_alarm = now + PRES_DELTA;
    }
    // if (now > tf_alarm) {
    //   if (tfmini.available()) {
    //     imu.lidar = tfmini.getDistance();
    //     // tf.distance = tfmini.getDistance();
    //     // tf.timestamp = now;
    //     // yivo->pack(MSG_DISTANCE, reinterpret_cast<uint8_t *>(&tf), sizeof(distance_t));
    //     // serial->write(yivo->get_buffer(), yivo->get_total_size());
    //     tf_alarm = now + TFM_DELTA;
    //   }
    // }

    return ret; // did we update imu struct?
  }

  // void set(Stream &s) { serial = &s; }
  // void set(Yivo &y) { yivo = &y; }

  // distance_t tf;
  imu_full_t imu;

  protected:
  // Yivo* yivo;
  // Stream* serial;
  QCF qcf;
  LSM6DSOX::gciLSM6DSOX sox;   // accel and gyro
  LIS3MDL::gciLIS3MDL lis;     // magnetometer
  BMP390::gciBMP390 bmp;       // pressure
  // TFmini tfmini;               // lidar
  uint32_t imu_alarm;
  // uint32_t tf_alarm; 
  uint32_t epoch;
  uint32_t pres_alarm;
};