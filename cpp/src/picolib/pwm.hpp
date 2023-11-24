
#pragma once


/*

THIS HAS SUCKY PERFORMANCE!!!!!

*/

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// https://github.com/107-systems/107-Arduino-Servo-RP2040/tree/main
constexpr uint16_t SERVO_MAX_PULSE_WIDTH = 2000UL;
constexpr uint16_t SERVO_MID_PULSE_WIDTH = 1500UL;
constexpr uint16_t SERVO_MIN_PULSE_WIDTH = 1000UL;
// constexpr uint16_t SERVO_PERIOD_USEC      = 20 * 1000UL;

class Servo {
  uint32_t slice_num{0};
  // uint channel{0};
  uint32_t pin{0};
  uint16_t max_us;
  uint16_t min_us;
  const uint32_t clkDiv{64};
  const uint16_t wrap{39063};

public:
  Servo() {}
  ~Servo() {}

  void init(const uint32_t pwm_pin, uint16_t max_pwm_us = SERVO_MAX_PULSE_WIDTH,
            uint16_t min_pwm_us = SERVO_MIN_PULSE_WIDTH) {

    pin = pwm_pin;
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    // Determine slice/channel pin belongs to
    slice_num = pwm_gpio_to_slice_num(pwm_pin);
    // channel   = pwm_gpio_to_channel(pwm_pin);

    float max = 2400, min = 600;
    max_us = uint16_t((max - min) / 20E3 * wrap);
    min_us = uint16_t(min / 20E3 * wrap);

    // // Configure GPIO for output
    // gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    // // Set system clock to 1 MHz. This is a time increment of 1 us
    // pwm_set_clkdiv_int(slice_num, 64);
    // // Set system period to ~20 ms
    // pwm_set_wrap(slice_num, 39063);
    // // Set initial output value to 1.5 ms
    // pwm_set_chan_level(slice_num, channel, SERVO_MID_PULSE_WIDTH);
    // // Enable PWM
    // pwm_set_enabled(slice_num, true);


    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, clkDiv);
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);
  }

  void write_us(uint16_t pulse_width_us) {
    if (pulse_width_us > max_us) pulse_width_us = max_us;
    // if (pulse_width_us < min_us) pulse_width_us = min_us;
    // pwm_set_chan_level(slice_num, channel, pulse_width_us);
    // pwm_set_gpio_level(pin, (pulse_width_us/20000.f)*wrap);
    pwm_set_gpio_level(pin, pulse_width_us + min_us); // helper, might be better to get slice/channel
    // pwm_set_gpio_level(pin, pulse_width_us + 1172); // helper, might be better to get slice/channel
  }
};

using ESC = Servo; // just an alias ... want to do better esc stuff later

// class ESC: public Servo {
//   public:
// };