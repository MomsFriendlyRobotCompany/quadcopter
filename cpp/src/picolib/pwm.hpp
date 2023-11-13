
#pragma once

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// https://github.com/107-systems/107-Arduino-Servo-RP2040/tree/main
constexpr uint16_t SERVO_MAX_PULSE_WIDTH  = 2000UL;
constexpr uint16_t SERVO_ZERO_PULSE_WIDTH = 1500UL;
constexpr uint16_t SERVO_MIN_PULSE_WIDTH  = 1000UL;
constexpr uint16_t SERVO_PERIOD_USEC      = 20 * 1000UL;

class Servo {
  uint slice_num{0};
  uint channel{0};
  uint16_t max_us;
  uint16_t min_us;

public:
  Servo() {}
  ~Servo() {}

  void init(const uint pwm_pin, uint16_t max_pwm_us = SERVO_MAX_PULSE_WIDTH,
            uint16_t min_pwm_us = SERVO_MIN_PULSE_WIDTH) {
    uint32_t const SYS_CLK_hz = clock_get_hz(clk_sys);
    uint32_t const PWM_CLK_hz = 1 * 1000UL * 1000UL;
    float const clk_divider =
        static_cast<float>(SYS_CLK_hz) / static_cast<float>(PWM_CLK_hz);

    // Determine slice/channel pin belongs to
    slice_num = pwm_gpio_to_slice_num(pwm_pin);
    channel   = pwm_gpio_to_channel(pwm_pin);
    max_us = max_pwm_us;
    min_us = min_pwm_us;

    // Configure GPIO for output
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    // Set system clock to 1 MHz. This is a time increment of 1 us
    pwm_set_clkdiv(slice_num, clk_divider);
    // Set system period to 20 ms
    pwm_set_wrap(slice_num, SERVO_PERIOD_USEC);
    // Set initial output value to 1.5 ms
    pwm_set_chan_level(slice_num, channel, SERVO_ZERO_PULSE_WIDTH);
    // Enable PWM
    pwm_set_enabled(slice_num, true);
  }

  void write_us(uint16_t pulse_width_us) {
    if (pulse_width_us > max_us) pulse_width_us = max_us;
    if (pulse_width_us < min_us) pulse_width_us = min_us;
    pwm_set_chan_level(slice_num, channel, pulse_width_us);
  }
};

using ESC = Servo; // just an alias ... want to do better esc stuff later

// class ESC: public Servo {
//   public:
// };