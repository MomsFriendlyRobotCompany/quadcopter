
#pragma once

#include "../defs.hpp"
#include "../memory.hpp"
#include <mavlink.h>

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

void mav_heartbeat(uint8_t *buffer) {
  mavlink_message_t message;

  const uint8_t system_id = 42;
  const uint8_t base_mode = 0;
  const uint8_t custom_mode = 0;
  mavlink_msg_heartbeat_pack_chan(
    system_id,
    MAV_COMP_ID_PERIPHERAL,
    MAVLINK_COMM_0,
    &message,
    MAV_TYPE_GENERIC,
    MAV_AUTOPILOT_GENERIC,
    base_mode,
    custom_mode,
    MAV_STATE_STANDBY);

  // uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const int len = mavlink_msg_to_send_buffer(buffer, &message);
}

void mav_attitude(uint8_t *buffer) {
  ;
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
  ;
}