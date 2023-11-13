
# PWM Servos

- 16b counter
- 8.4 fractional clock divider
- 2 slices, 8 channels for 16 independent PWM signals

- [ref](https://vanhunteradams.com/Pico/Helicopter/MotorCircuit.html)

```c
#include "hardware/pwm.h"

// Midpoint of potentiometer
#define WRAPVAL 25000
#define CLKDIV 5.0f
#define PWM_PIN 5

// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;

void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));
}

int main () {
    // Tell GPIO 5 that it is allocated to the PWM
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    while (1) { ... }
}
```

