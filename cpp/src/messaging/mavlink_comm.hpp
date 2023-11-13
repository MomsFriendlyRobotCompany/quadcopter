
#pragma once

#include <mavlink.h>

void mav_heartbeat() {
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

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const int len = mavlink_msg_to_send_buffer(buffer, &message);
}