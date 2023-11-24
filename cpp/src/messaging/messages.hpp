
#pragma once

#include <cstdint>


constexpr uint8_t MSG_PING        = 10;
constexpr uint8_t MSG_HEARTBEAT   = 11;
constexpr uint8_t MSG_BATTERY     = 12;
constexpr uint8_t MSG_CALIBRATION = 13;
constexpr uint8_t MSG_IMU = 14;
constexpr uint8_t MSG_GPS = 15;
constexpr uint8_t MSG_POSE = 16;

struct __attribute__((packed)) heartbeat_t {
  enum Status: uint8_t {
    OFF=0,
    STANDBY=1,
    ARMED=2
  };

  uint32_t timestamp;
  uint8_t status;
};

struct __attribute__((packed)) date_t {
  uint8_t year, month, day;
}; // 3*1 = 3

struct __attribute__((packed)) vec_t {
  float x,y,z;
}; // 3*4 = 12

struct __attribute__((packed)) quat_t {
  float w,x,y,z;
}; // 4*4 = 16

struct __attribute__((packed)) clock_time_t {
  uint8_t hour, minute;
  float seconds;
}; // 2 + 4 = 7

struct __attribute__((packed)) satnav_t {
  float lat, lon; // decimal degrees
  float altitude; // meters above MSL
  float hdop; // horizontal dilution of precision
  uint8_t satellites;
  uint8_t fix;
  date_t date;
  clock_time_t time;
}; // 16+2+3+6 = 27


struct __attribute__((packed)) imu_agmpt_t {
  vec_t a;  // 12 [0:11]
  vec_t g;  // 12 [12:23]
  vec_t m;  // 12 [24:35]
  float temperature;  // 4 [36:39]
  float pressure;     // 4 [40:43]
  uint32_t timestamp; // 4 [44:47] - FIXME: u32 (arduino) but u64 linux/mac
}; // 36+8+4 = 48


struct __attribute__((packed)) pose_t {
  vec_t pos;
  vec_t vel;
  quat_t q;
  uint32_t timestamp;
};