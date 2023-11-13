
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "tusb.h" // wait for USB
#include <stdio.h>

#include "picolib/pwm.hpp"

constexpr uint SERVO_PIN = 16;
constexpr uint32_t SLEEP = 100;

int main() {
  stdio_init_all();

  // wait for USB serial connection
  // while (!tud_cdc_connected()) {
  //   sleep_ms(100);
  // }

  Servo s;
  s.init(SERVO_PIN);
  bi_decl(bi_1pin_with_name(SERVO_PIN, "Servo pin"));

  printf("CW: ");

  while (1) {
    for (uint16_t us = 1000; us <= 2000; us += 20) {
      s.write_us(us);
      sleep_ms(SLEEP);
      printf(".");
    }

    printf("\nCCW:");

    for (uint16_t us = 2000; us >= 1000; us -= 20) {
      s.write_us(us);
      sleep_ms(SLEEP);
      printf(".");
    }

    printf("\nCW: ");
  }

  return 0;
}