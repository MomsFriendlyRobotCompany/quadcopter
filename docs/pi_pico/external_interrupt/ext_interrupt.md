# External Interrupt

```c
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define the GPIO pin for the external interrupt
#define YOUR_GPIO_PIN 15

// External interrupt handler function
void my_interrupt_handler() {
    // Your interrupt handling code goes here
    // This function will be called when the interrupt is triggered
    printf("External interrupt triggered!\n");
}

int main() {
    stdio_init_all();

    // Initialize the GPIO pin for the external interrupt
    gpio_init(YOUR_GPIO_PIN);
    gpio_set_dir(YOUR_GPIO_PIN, GPIO_IN);
    gpio_pull_up(YOUR_GPIO_PIN);

    // Set up the interrupt handler
    gpio_set_irq_enabled(YOUR_GPIO_PIN, GPIO_IRQ_EDGE_RISE, true);
    irq_set_exclusive_handler(IO_IRQ_BANK0, my_interrupt_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Main loop
    while (1) {
        // Your main code goes here
    }

    return 0;
}
```