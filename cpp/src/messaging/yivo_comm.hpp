
#pragma once

#include "messages.hpp"
#include "../memory.hpp"
#include <yivo.hpp>
#include "../picolib/picolib.hpp" // write_stdout, read_stdin


static
void yivo_heartbeat(const heartbeat_t& msg) {
  yivo::yivopkt_t y;
  y.pack(MSG_HEARTBEAT, (uint8_t*)&msg, sizeof(msg));
  write_stdout(y.data(), y.size());
}

static
void yivo_imu(const imu_agmpt_t& msg) {
  yivo::yivopkt_t y;
  y.pack(MSG_IMU, (uint8_t*)&msg, sizeof(msg));
  write_stdout(y.data(), y.size());
}

static
void yivo_gps(const satnav_t& msg) {
  yivo::yivopkt_t y;
  y.pack(MSG_GPS, (uint8_t*)&msg, sizeof(msg));
  write_stdout(y.data(), y.size());
}

constexpr uint8_t buffer_size = 64;

static
void get_inputs() {
  uint8_t buffer[buffer_size];
  yivo::Parser yivo;
  uint32_t num = 0;
  while (num != PICO_ERROR_TIMEOUT) {
    uint8_t id = 0;
    num = read_stdin(buffer, buffer_size, 0);
    for (uint32_t i=0; i< buffer_size; ++i) {
      uint8_t b = buffer[i];
      id = yivo.parse(b);
      if (id > 0) break;
    }

    if (id > 0) {
      yivo::yivopkt_t p;
      yivo.get_packet(p);

      if (id == MSG_CALIBRATION) {}
      else if (id == MSG_INTERVAL) {}
    }
  }
}