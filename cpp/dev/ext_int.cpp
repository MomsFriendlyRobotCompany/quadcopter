
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
// #include <rp2040.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define EXT_INT_PIN 25

// Create a function to handle the interrupt
void irq_handler(uint gpio, uint32_t event_mask) {
  // Do something when the interrupt occurs
  printf("Interrupt received!\n");
}

int main() {
  // Initialize the GPIO pin
  gpio_init(EXT_INT_PIN);
  gpio_set_dir(EXT_INT_PIN, GPIO_IN);

  gpio_set_irq_enabled_with_callback(EXT_INT_PIN, GPIO_IRQ_EDGE_RISE, true,
                                     irq_handler);

  // Wait for interrupts
  while (1) {
    sleep_ms(1000);
  }

  return 0;
}