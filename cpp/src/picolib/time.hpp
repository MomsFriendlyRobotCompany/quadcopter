
#pragma once

#include <cstdint>
#include "pico/time.h"

static
uint64_t time_since_boot_us() {
  absolute_time_t t = get_absolute_time();
  return to_us_since_boot(t);
}

static
uint32_t time_since_boot_ms() {
  absolute_time_t t = get_absolute_time();
  return to_ms_since_boot(t);
}