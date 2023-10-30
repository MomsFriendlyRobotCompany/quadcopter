
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "tusb.h" // wait for USB


volatile bool led_status = false;
volatile uint count = 1;
const uint LED_PIN = 25;


bool callback_100hz(struct repeating_timer *t) {
  // printf("Repeat at %lld\n", time_us_64());
  if ((count++ % 100) == 0) {
    led_status = !led_status;
    gpio_put(LED_PIN, led_status);
    count = 1;
  }

  return true;
}

bool callback_1hz(struct repeating_timer *t) {
  printf(".");
  return true;
}

int main() {
  stdio_init_all();

  // wait for USB serial connection
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  printf("Hello Timer!\n");
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  bi_decl(bi_1pin_with_name(LED_PIN, "LED pin"));

  // Create a repeating timer that calls repeating_timer_callback.
  // If the delay is > 0 then this is the delay between the previous callback ending and the next starting.
  // If the delay is negative (see below) then the next call to the callback will be exactly 500ms after the
  // start of the call to the last callback
  struct repeating_timer timer_100hz;
  struct repeating_timer timer_1hz;

  // Negative delay so means we will call repeating_timer_callback, and call it again
  // 500ms later regardless of how long the callback took to execute
  add_repeating_timer_ms(-10, callback_100hz, NULL, &timer_100hz);
  add_repeating_timer_ms(-1000, callback_1hz, NULL, &timer_1hz);

  while (1) {
    sleep_ms(1000);
    // tight_loop_contents();
  }

  return 0;
}