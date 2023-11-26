
#pragma once

#include "messages.hpp"
#include "../memory.hpp"
#include <yivo.hpp>
#include "../picolib/picolib.hpp"

// extern Serial0;

static
void yivo_heartbeat() {
  yivopkt_t msg;
  heartbeat_t hb {time_since_boot_ms(), 0};
  msg.pack(MSG_HEARTBEAT, (uint8_t*)&hb, sizeof(hb));

  write_stdout(msg.data(), msg.size());
}