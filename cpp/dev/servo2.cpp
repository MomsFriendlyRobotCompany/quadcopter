
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define PWM_PIN 2
#define PWM_RANGE                                                              \
  10000 // Adjust this value based on your servo's specifications

void set_servo_angle(int angle) {
  int pulse_width =
      (int)((angle / 180.0) * PWM_RANGE) + 500; // Convert angle to pulse width
  pwm_set_gpio_level(PWM_PIN, pulse_width);
}

int main() {
  stdio_init_all();

  gpio_init(PWM_PIN);
  gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);

  pwm_set_wrap(slice_num, PWM_RANGE);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);

  pwm_set_enabled(slice_num, true);

  while (1) {
    set_servo_angle(0); // Move to 0 degrees
    sleep_ms(1000);
    set_servo_angle(90); // Move to 90 degrees
    sleep_ms(1000);
    set_servo_angle(180); // Move to 180 degrees
    sleep_ms(1000);
  }

  return 0;
}
