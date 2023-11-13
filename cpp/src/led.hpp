
#pragma once

#include "blink.pio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
  blink_program_init(pio, sm, offset, pin);
  pio_sm_set_enabled(pio, sm, true);

  printf("Blinking pin %d at %d Hz\n", pin, freq);

  // PIO counter program takes 3 more cycles in total than we pass as
  // input (wait for n + 1; mov; jmp)
  pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void led_init() {
  PIO pio     = pio0;
  uint offset = pio_add_program(pio, &blink_program);
  // printf("Loaded program at %d\n", offset);
  blink_pin_forever(pio, 0, offset, LED_PIN, 1);
}