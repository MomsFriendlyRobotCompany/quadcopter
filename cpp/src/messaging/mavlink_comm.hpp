
#pragma once

#include "../defs.hpp"
#include "../memory.hpp"
// #include <mavlink.h>

/*
once:
  autopilot_verion - misc

1 hz:
  heartbeat - misc
  sys_status - sensor_info/batter_info/comm_errs/other_errs

2-5 hz:
  battery_status - too much info
  global_position_ned - lat/lon/alt/above_ground/heading

10-25 hz:
  attitude_quaternion - quaternion/rpy_rates
  local_position_ned - pos/vel

100 hz:
  highres_imu - a,g,m,press,temp,alt

*/

constexpr uint8_t sys_id = 1;
constexpr uint8_t comp_id = 1;

extern SharedMemory_t memory;

void mav_heartbeat(uint8_t *buffer) {
  // mavlink_message_t message;

  // const uint8_t base_mode = 0;
  // const uint8_t custom_mode = 0;
  // mavlink_msg_heartbeat_pack(
  //   sys_id,
  //   MAV_COMP_ID_PERIPHERAL,
  //   &message,
  //   MAV_TYPE_GENERIC,
  //   MAV_AUTOPILOT_GENERIC,
  //   base_mode,
  //   custom_mode,
  //   MAV_STATE_STANDBY);

  // uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  // const int len = mavlink_msg_to_send_buffer(buffer, message);
}

void mav_attitude(uint8_t *buffer) {
  // mavlink_message_t message;
  // float q[4]{0,0,0,0};

  // mavlink_msg_attitude_quaternion_pack(
  //   sys_id,
  //   comp_id,
  //   message,
  //   time_since_boot_us(), // time
  //   memory.q.w,
  //   memory.q.x,
  //   memory.q.y,
  //   memory.q.z,
  //   memory.imu.g.x,
  //   memory.imu.g.y,
  //   memory.imu.g.z,
  //   q
  // );
  // const int len = mavlink_msg_to_send_buffer(buffer, message);
}

void mav_local_position(uint8_t *buffer) {
  ;
}

void mav_global_position(uint8_t *buffer) {
  ;
}

void mav_battery(uint8_t *buffer) {
  ;
}

void mav_imu(uint8_t *buffer) {
  // mavlink_message_t message;
  // mavlink_msg_highres_imu_pack(
  //   sys_id,
  //   comp_id,
  //   &message,
  //   0, // time u64
  //   memory.imu.a.x,
  //   memory.imu.a.y,
  //   memory.imu.a.z,
  //   memory.imu.g.x,
  //   memory.imu.g.y,
  //   memory.imu.g.z,
  //   memory.mags.x,
  //   memory.mags.y,
  //   memory.mags.z,
  //   0, // press alt
  //   memory.press_temp.press,
  //   0, // diff press
  //   memory.press_temp.temp,
  //   65535, // fields
  //   0 // id=0
  // );
}