#pragma once


#include <gciSensors.hpp>
#include <yivo.hpp>
#include <gecko2/messages.hpp>
#include <squaternion.hpp>
#include "MadgwickAHRS.hpp"

// extern float q0, q1, q2, q3;

class SensorSuite {
  public:
  SensorSuite (): sox(&Wire), lis(&Wire), bmp(&Wire), qcf(0.02) {}
      //yivo(nullptr), serial(nullptr) {}

  bool init() {
    // tfmini.attach(Serial1);
    // tf.min_distance = 2;
    // tf.max_distance = 1200;
    // tf.type = distance_t::LIDAR;
    epoch = 0; //millis();

    bool ok = true;

    ok &= sox.init(
      LSM6DSOX::ACCEL_RANGE_4_G,
      LSM6DSOX::GYRO_RANGE_2000_DPS,
      LSM6DSOX::RATE_104_HZ);
    ok &= lis.init(
      LIS3MDL::RANGE_4GS,
      LIS3MDL::ODR_155HZ);
    ok &= bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X);
    // bmp.setOsMode(BMP390::OS_MODE_PRES_16X_TEMP_2X);
    // bmp.setPowerMode(BMP390::MODE_NORMAL); // continuous sampling
    // for (int i; i< 10; ++i) {
    //   bmp.read();
    //   delay(40); // 25 Hz
    // }
    while (!sox.ready());
    LSM6DSOX::sox_t s = sox.read();
    epoch = s.ts;

    imu.lidar = 0.0f;

    return ok;
  }

  bool check() {
    bool soxread = false;
    bool lisread = false;
    float dt;

    if (sox.ready()) {
      LSM6DSOX::sox_t s = sox.read();

      imu.accelleration.x = s.ax;
      imu.accelleration.y = s.ay;
      imu.accelleration.z = s.az;
      imu.gyroscope.x = s.gx;
      imu.gyroscope.y = s.gy;
      imu.gyroscope.z = s.gz;
      imu.timestamp = s.ts;

      // uint32_t now = millis();
      // float dt = 0.001*(now - epoch); // good enough?
      // epoch = now;

      // FIXME: handle rollover
      dt = (s.ts - epoch) * 25e-6;
      epoch = s.ts;

      // Quaternion q = qcf.update(
      //   s.ax, s.ay, s.az,
      //   s.gx, s.gy, s.gz,
      //   dt);
      // MadgwickAHRSupdate(
      //   s.gx, s.gy, s.gz,
      //   s.ax, s.ay, s.az,
      // );
  
      // imu.orientation.w = q.w;
      // imu.orientation.x = q.x;
      // imu.orientation.y = q.y;
      // imu.orientation.z = q.z;

      soxread = true;
    }

    if (lis.ready()) {
      LIS3MDL::mag_t l = lis.read();
      imu.magnetometer.x = l.x;
      imu.magnetometer.y = l.y;
      imu.magnetometer.z = l.z;
      lisread = true;
    }

    if (bmp.ready()) {
      BMP390::pt_t b = bmp.read();
      imu.altitude = bmp.altitude(b.press);
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

    float q0, q1, q2, q3;

    if (soxread && lisread) {
      MadgwickUpdate(
        imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z,
        imu.accelleration.x, imu.accelleration.y, imu.accelleration.z,
        imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z,
        dt
      );
      getQuaternion(q0,q1,q2,q3);
      imu.orientation.w = q0;
      imu.orientation.x = q1;
      imu.orientation.y = q2;
      imu.orientation.z = q3;
    }
    else if (soxread) {
      MahonyUpdate(
        imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z,
        imu.accelleration.x, imu.accelleration.y, imu.accelleration.z,
        dt
      );
      getQuaternion(q0,q1,q2,q3);
      imu.orientation.w = q0;
      imu.orientation.x = q1;
      imu.orientation.y = q2;
      imu.orientation.z = q3;
    }

    return soxread; // did we update imu struct?
  }

  imu_full_t imu;

  protected:
  QCF qcf;
  LSM6DSOX::gciLSM6DSOX sox;   // accel and gyro
  LIS3MDL::gciLIS3MDL lis;     // magnetometer
  BMP390::gciBMP390 bmp;       // pressure
  // TFmini tfmini;               // lidar
  uint32_t epoch;
};