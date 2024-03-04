
#pragma once

#include <cstdint>
#include "memory.hpp"

extern SharedMemory_t memory;

// System States
constexpr uint8_t SS_BOOT      = 0;
constexpr uint8_t SS_IDLE     = 1;
// constexpr uint8_t SS_CALIBRATE = 2;
constexpr uint8_t SS_SAFE      = 4;
constexpr uint8_t SS_ARMED     = 8;
constexpr uint8_t SS_FLY     = 16;
// constexpr uint8_t SS_SHUTDOWN     = 16;

struct SystemState_t {
  uint8_t current_state = SS_BOOT;
  bool calibrated = false;
  bool armed = false;
  bool shutdown = false;
  bool takeoff = false; // fly?
  bool telemetry = false;
};

SystemState_t sysstate;

void handle_boot(SystemState_t& ss, SharedMemory_t& sm) {
  if (ss.calibrated == false) {
    // call calibrate
    ss.calibrated = true;
    ss.current_state = SS_IDLE;
    return;
  }
}

void handle_idle(SystemState_t& ss, SharedMemory_t& sm) {
  if (ss.calibrated == false) {
    // call calibrate
    ss.calibrated = true;
    ss.telemetry = true;
    ss.current_state = SS_IDLE;
    return;
  }
  else if (ss.shutdown == true) {
    ss.current_state = SS_SAFE;
  }
  else if (ss.armed == true) {
    // activate ESC
    ss.current_state = SS_ARMED;
  }
}

void handle_armed(SystemState_t& ss, SharedMemory_t& sm) {
  if (ss.takeoff == true) {
    // fly
    ss.current_state = SS_FLY;
  }
  else if (ss.armed == false) {
    // land
    // deactivate ESC
    ss.current_state = SS_IDLE;
  }
}

void handle_safe(SystemState_t& ss, SharedMemory_t& sm) {
  // stop telemetry
  // stop motors
  // no recovery, it is done
  // while (true) {
  //   sleep_ms(1000);
  //   wdog.touch();
  // }
}

void update_state(SystemState_t& ss) {
  if (ss.current_state == SS_BOOT) handle_boot(ss, memory);
  else if (ss.current_state == SS_IDLE) handle_idle(ss, memory);
  else if (ss.current_state == SS_ARMED) handle_armed(ss, memory);
  else if (ss.current_state == SS_FLY) handle_armed(ss, memory);
  else if (ss.current_state == SS_SAFE) handle_safe(ss, memory);
}