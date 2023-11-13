
#pragma once

#include "defs.hpp"
#include "memory.hpp"
#include "messaging/mavlink_comm.hpp"

void main_core_1() {
  mav_heartbeat();
  while(1) sleep_ms(2000);
}