
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "tusb.h" // wait for USB
#include <stdio.h>

#include "picolib/pwm.hpp"

constexpr uint SERVO_PIN = 16;

int main() {
  stdio_init_all();

  // wait for USB serial connection
  // while (!tud_cdc_connected()) {
  //   sleep_ms(100);
  // }

  Servo s;
  s.init(SERVO_PIN);
  bi_decl(bi_1pin_with_name(SERVO_PIN, "Servo pin"));

  bool dir = true;
  uint16_t us = 0;
  while (1) {
    if (us > 3500) dir = false;
    if (us < 10) dir = true;
    us += (dir) ? 50 : -50;
    s.write_us(us);
    sleep_ms(10);
    // printf(".");
  }

  return 0;
}