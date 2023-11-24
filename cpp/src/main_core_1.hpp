
#pragma once

#include "defs.hpp"
#include "memory.hpp"
#include "messaging/mavlink_comm.hpp"

/*
Core 0:
- read sensors via I2C
- calculate control and set motors
- handle usb debug

Core 1:
- I "think" this will run my compre complex navigation code (which is slower)
- publishing mavlink/yivo messages
*/

/*
 Need to setup a timer pool here for publishing
 messages at certain rates:
  heartbeat - 1hz
  gps - 1hz
  pos/vel/attitude - 10hz
  battery - 5 hz
  esc_info - 10hz
*/

extern SharedMemory_t memory;
extern Mutex sm_mutex;

void update_ins_solution() {
  sm_mutex.acquire();
  sleep_us(20);
  sm_mutex.release();

  sleep_ms(90); // do math
  sleep_ms(10); // pub various messages
}

void main_core_1() {
// #if defined(MSG_MAVLINK)
//   uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // > 256 ... some big number
// #else
//   uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // TODO: figure out how big
// #endif

// #if defined(MSG_MAVLINK)
//   mav_heartbeat(buffer);
// #else
//   yivo_heartbeat();
// #endif

  while (1) {
    update_ins_solution();
  }
}