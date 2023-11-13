
#pragma once

#include <gcigps.hpp>
#include <gciSensors.hpp>
#include <squaternion.hpp>

#include "flags.hpp"

using gci::sensors::vecf_t;

// different reoccuring timers
constexpr uint32_t TIMER_1HZ = (1 << 0);
constexpr uint32_t TIMER_5HZ = (1 << 1);
constexpr uint32_t TIMER_10HZ = (1 << 2);
constexpr uint32_t TIMER_50HZ = (1 << 3);
constexpr uint32_t TIMER_100HZ = (1 << 4);

// different sensors
constexpr uint16_t STATUS_ACCELS = (1 << 1);
constexpr uint16_t STATUS_GYROS = (1 << 2);
constexpr uint16_t STATUS_MAGS = (1 << 3);
constexpr uint16_t STATUS_POS = (1 << 4);
constexpr uint16_t STATUS_VEL = (1 << 5);
constexpr uint16_t STATUS_PRESS = (1 << 6);
constexpr uint16_t STATUS_TEMP = (1 << 7);
constexpr uint16_t STATUS_GPS = (1 << 8);
constexpr uint16_t STATUS_BATTERY = (1 << 9);

// constexpr uint16_t SET_BITS(const uint16_t val, const uint16_t mask) { return val | mask; }
// constexpr uint16_t CLEAR_BITS(const uint16_t val, const uint16_t mask) { return val & ~mask; }

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

  // BoolFlag timer100hz;
  // BoolFlag timer50hz;
  // BoolFlag timer10hz;
  // BoolFlag timer1hz;

  // uint16_t status; // which values

  BitFlag timers;
  BitFlag sensors;
};

static
SharedMemory_t memory;