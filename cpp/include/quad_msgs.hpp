
#pragma once

#include <messages.hpp>

namespace quad {

constexpr uint8_t MOTORS = 127;
constexpr uint8_t CMD_MOTORS = 128;
constexpr uint8_t IMU = 129;
constexpr uint8_t CALIBRATION = 130;
constexpr uint8_t CONTINUE = 131;
constexpr uint8_t HEARTBEAT = 132;
constexpr uint8_t REBOOT = 133;
// constexpr uint8_t REBOOT = 133;

enum SensorTypes: uint16_t {
  ACCELEROMETER=1,
  GYROSCOPE=2,
  MAGNOMETER=4,
  BAROMETER=8,
  RANGER=16,
  GPS=32
};

struct __attribute__((packed)) continue_t {
  uint8_t ok;
};

struct __attribute__((packed)) heartbeat_t {
  uint8_t version;
  uint8_t state;      // platform specific - idle, flying, etc ...
  uint8_t health;     // is anything broken?
  uint16_t sensors;   // what is onboard?
  uint16_t battery;   // battery voltage
  uint32_t timestamp;
};

// too specific ...
// struct __attribute__((packed)) joystick_t {
//   uint16_t ok; // twist?
// };

struct __attribute__((packed)) calibration_sensors_t {
  float a[12]; // accels
  float g[3];  // gyros
  float m[6];  // mags
};

struct __attribute__((packed)) motor_cmd_t {
  uint16_t a,b,c,d;
};

struct __attribute__((packed)) motor_status_t {
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

struct __attribute__((packed)) imu_t: accel_t, gyro_t, mag_t {
  enum IMU_Status: uint8_t {
    OK      = 0,
    A_FAIL  = 1,
    G_FAIL  = 2,
    M_FAIL  = 4,
    PT_FAIL = 8
  };
  uint8_t status;
  long timestamp;
};

} // end namespace quadcopter