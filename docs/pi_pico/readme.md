# Pico

![](./pico-pinout.png)

![](./pico-w-pinout.png)

- [Arudino-Pico readthedocs](https://arduino-pico.readthedocs.io/en/latest/index.html)
- [Raspberry Pi Pico and Pico W Pinout Guide](https://randomnerdtutorials.com/raspberry-pi-pico-w-pinout-gpios/)

## Serial

- `Serial1` / `uart0` uses TX(`0`) / RX(`1`)
- `Serial2` / `uart1` uses TX(`8`) / RX(`9`)

```cpp
// call these before begin
Serial1.setRX(pinRX);
Serial1.setTX(pinTX);
Serial1.setFIFOSize(128); // default 32
Serial1.begin(baud);
```


## ADC

- ADC on pins 26, 27, 28
- `ADC3` measures `VSYS`
- `ADC4` measures CPU temperature sensor

## Pins

- `RUN` set `LOW` to reset the uC
- `VSYS` is the external power input which is probably the USB and ranges between 1.8V-5.5V
- `VBUS` is power from the USB input, typicall 5V

You can tell `picotool` what some of the pins do:

```c
enum gpio_function {
    GPIO_FUNC_XIP = 0,
    GPIO_FUNC_SPI = 1,
    GPIO_FUNC_UART = 2,
    GPIO_FUNC_I2C = 3,
    GPIO_FUNC_PWM = 4,
    GPIO_FUNC_SIO = 5,
    GPIO_FUNC_PIO0 = 6,
    GPIO_FUNC_PIO1 = 7,
    GPIO_FUNC_GPCK = 8,
    GPIO_FUNC_USB = 9,
    GPIO_FUNC_NULL = 0xf,
};

// INFO ////////////////////////////////////////////////////////////
bi_decl(bi_program_description("something"));
bi_decl(bi_program_version_string("v1"));
bi_decl(bi_program_url("github"));
// I2C ------------------------------------------------------------
bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C));
// UART -----------------------------------------------------------
bi_decl(bi_2pins_with_func(UART0_RX_PIN, UART0_TX_PIN, GPIO_FUNC_UART));
bi_decl(bi_2pins_with_func(UART1_RX_PIN, UART1_TX_PIN, GPIO_FUNC_UART));
// ADC ------------------------------------------------------------
bi_decl(bi_1pin_with_name(ADC_BATT_PIN, "Battery ADC"));
// PWM ------------------------------------------------------------
bi_decl(bi_1pin_with_name(pwm_m0, "PWM M0"));
bi_decl(bi_1pin_with_name(pwm_m1, "PWM M1"));
bi_decl(bi_1pin_with_name(pwm_m2, "PWM M2"));
bi_decl(bi_1pin_with_name(pwm_m3, "PWM M3"));
// LED ------------------------------------------------------------
bi_decl(bi_1pin_with_name(LED_PIN, "LED"));
////////////////////////////////////////////////////////////////////
```

## Time

```c
absolute_time_t get_absolute_time(); // returns absolute_time_t which is struct{uint64_t}
uint64_t to_us_since_boot(absolute_time_t);
uint32_t to_ms_since_boot(absolute_time_t);
```

```c
uint64_t time_since_boot_us() {
    absolute_time_t t = get_absolute_time();
    return to_us_since_boot(t);
}

uint32_t time_since_boot_ms() {
    absolute_time_t t = get_absolute_time();
    return to_ms_since_boot(t);
}
```