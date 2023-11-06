
#pragma once

#include <gcigps.hpp>
#include <gciSensors.hpp>

#include "trigger.hpp"

using gci::sensors::vecf_t;


constexpr uint16_t STATUS_ACCELS = (1 << 1);
constexpr uint16_t STATUS_GYROS = (1 << 2);
constexpr uint16_t STATUS_MAGS = (1 << 3);
constexpr uint16_t STATUS_POS = (1 << 4);
constexpr uint16_t STATUS_VEL = (1 << 5);
constexpr uint16_t STATUS_PRESS = (1 << 6);
constexpr uint16_t STATUS_TEMP = (1 << 7);
constexpr uint16_t STATUS_GPS = (1 << 8);
constexpr uint16_t STATUS_BATTERY = (1 << 9);

constexpr uint16_t SET_BITS(const uint16_t val, const uint16_t mask) { return val | mask; }
constexpr uint16_t CLEAR_BITS(const uint16_t val, const uint16_t mask) { return val & ~mask; }

struct SharedMemory_t {
  vecf_t accels;
  vecf_t gyros;
  vecf_t mags;
  vecf_t pos;
  vecf_t vel;
  float pressure;
  float temperature;
  gga_t gps;
  float battery;

  Trigger timer100hz;
  Trigger timer1hz;
  // bool

  uint16_t status; // which is better?
};

static
SharedMemory_t memory;