// #pragma once

// #include <gciSensors.hpp>
// #include <squaternion.hpp>
// #include <yivo.hpp>
// #include <gecko2/sensor.hpp>
// #include <gecko2/messages.hpp>

// class gciLSOXLISBMP : public cSensor {
// public:
//   gciLSOXLISBMP()
//       : soxFound(false), magFound(false), pressFound(false),
//         qcf(0.02), sm{{1.00268927, -0.00056029, -0.00190925, -0.00492348},
//                       {-0.00138898, 0.99580818, -0.00227335, 0.00503835},
//                       {-0.01438271, 0.00673172, 0.9998954, -0.01364759}},
//         gbias{-0.00889949 - 0.00235061 - 0.00475294},
//         mbias{-13.15340002, 29.7714855, 0.0645215}, mm{0.96545537, 0.94936676,
//                                                        0.967698},
//         sox(&Wire), lis3mdl(&Wire), bmp(&Wire) {}

//   bool init() {
//     if (sox.init()) {
//       soxFound = true;

//       // Accelerometer ------------------------------------------
//       // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
//       // sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

//       // Gyros ----------------------------------------------------
//       // sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
//       // sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
//     }

//     // Magnetometer -----------------------------------------------------
//     if (lis3mdl.init()) {
//       magFound = true;
//       // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
//       // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
//       // already does this
//       // lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
//       // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
//       // lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
//     }

//     if (bmp.init()) {
//       pressFound = true;
//     }

//     found = magFound && soxFound && pressFound;
//     ts    = millis();
//     return found;
//   }

//   void read() {

//     if (soxFound) {
//       LSM6DSOX::sox_t s = sox.read();

//       // #if IMU_USE_UNCALIBRATED_DATA
//       // msg.ax = s.ax; // g
//       // msg.ay = s.ay;
//       // msg.az = s.az;
//       // #else
//       // msg.ax = sm[0][0] * s.ax + sm[0][1] * s.ay + sm[0][2] * s.az +
//       // sm[0][3]; msg.ay = sm[1][0] * s.ax + sm[1][1] * s.ay + sm[1][2] * s.az
//       // + sm[1][3]; msg.az = sm[2][0] * s.ax + sm[2][1] * s.ay + sm[2][2] *
//       // s.az + sm[2][3]; #endif

//       // #if IMU_USE_UNCALIBRATED_DATA
//       // msg.gx = s.gx; // rad/s
//       // msg.gy = s.gy;
//       // msg.gz = s.gz;
//       // #else
//       // msg.gx = (s.gx - gbias[0]);
//       // msg.gy = (s.gy - gbias[1]);
//       // msg.gz = (s.gz - gbias[2]);
//       // #endif

//       msg.temperature = s.temp; // C

//       uint32_t now    = millis();
//       dt              = (now - ts) * 0.001;
//       ts              = now;

//       // q = qcf.update(
//       //   msg.ax, msg.ay, msg.az,
//       //   msg.gx, msg.gy, msg.gz,
//       //   dt);

//       // msg.qw = q.w;
//       // msg.qx = q.x;
//       // msg.qy = q.y;
//       // msg.qz = q.z;

//       msg.timestamp = now; // time msec
//     }

//     if (magFound) {
//       LIS3MDL::mag_t s = lis3mdl.read();

//       // #if IMU_USE_UNCALIBRATED_DATA
//       // msg.mx = s.x; // uT
//       // msg.my = s.y;
//       // msg.mz = s.z;
//       // #else
//       // msg.mx = mm[0] * s.x - mbias[0]; // uT
//       // msg.my = mm[1] * s.y - mbias[1];
//       // msg.mz = mm[2] * s.z - mbias[2];
//       // #endif
//     }

//     if (pressFound) {
//       BMP390::pt_t s = bmp.read();
//       // if (s.ok) {
//       //   msg.pressure = s.press;
//       //   msg.temperature = s.temp;
//       // }
//       // else {
//       //   msg.pressure = 0.0f;
//       //   msg.temperature = 0.0f;
//       // }
//     }
//   }

//   void get_msg(Yivo &y) {
//     y.pack(MSG_IMU_FULL, reinterpret_cast<uint8_t *>(&msg), sizeof(imu_full_t));
//   }

//   // const imu_full_t get_msg() const {
//   //   // if(!found) return;
//   //   // yivo.pack_n_send(IMU_AGMQPT, reinterpret_cast<uint8_t*>(&msg),
//   //   // sizeof(ImuAGMQPT_t)); func(100, reinterpret_cast<uint8_t*>(&msg),
//   //   // sizeof(ImuA_t));
//   //   return msg;
//   // }

// protected:
//   bool soxFound;
//   bool magFound;
//   bool pressFound;

//   // ImuAGMQPT_t msg;
//   imu_full_t msg;

//   QCF qcf;

//   LSM6DSOX::gciLSM6DSOX sox;   // accel and gyro
//   LIS3MDL::gciLIS3MDL lis3mdl; // magnetometer
//   BMP390::gciBMP390 bmp;       // pressure
//   Quaternion q;
//   float dt;       // time difference between samples
//   uint32_t ts;    // timestamp (msec)
//   float sm[3][4]; // accel scale/bias
//   float gbias[3]; // gyro bias
//   float mbias[3]; // mag bias
//   float mm[3];    // mag scale
// };
