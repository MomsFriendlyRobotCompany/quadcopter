/*
This will be a future library separate from quadcopter because there is
nothing here that is specific to quadcopter.
*/
#pragma once

#include <cstdint>

typedef uint32_t pin_t;

#include "adc.hpp"
#include "pwm.hpp"
#include "fifo.hpp"
#include "uart.hpp"
#include "spi.hpp"
#include "rtc.hpp"
#include "mutex.hpp"
#include "watchdog.hpp"