
#pragma once

namespace quad {

constexpr uint8_t MOTORS = 127;
constexpr uint8_t CMD_MOTORS = 128;
constexpr uint8_t IMU = 129;

struct __attribute__((packed)) cmd_motors_t {
  uint16_t a,b,c,d;
};


struct __attribute__((packed)) motors_t {
  enum status_t: uint8_t {
    OK,       // 0
    DEGRADED, // 1
    FAIL,     // 2
    UNKNOWN   // 3
  };
  uint16_t a,b,c,d; // pwm
  // uint16_t c0, c1, c2, c3; // current
  uint8_t armed;
  // status_t status; // bits: m3[6:7], m2[4:5], m1[2:3], m0[0:1]
};

enum class IMU_Status: uint8_t {
  OK      = 0,
  A_FAIL  = 1,
  G_FAIL  = 2,
  M_FAIL  = 4,
  PT_FAIL = 8
};
struct __attribute__((packed)) accel_t {
  vec_t accel; // 12 [0:11]
};
struct __attribute__((packed)) gyro_t {
  vec_t gyro;  // 12 [0:11]
};
struct __attribute__((packed)) mag_t {
  vec_t mag;  // 12 [0:11]
};
struct __attribute__((packed)) imu_t: accel_t,gyro_t,mag_t,atmospheric_t {
  uint8_t status;
  long timestamp;
};

} // end namespace quadcopter