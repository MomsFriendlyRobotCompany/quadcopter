#pragma once

// https://randomnerdtutorials.com/raspberry-pi-pico-w-pinout-gpios/


// #if defined(__SAMD51__)

constexpr int BOARD_SDA               = 4;
constexpr int BOARD_SCL               = 5;
constexpr int BOARD_RX                = 1;
constexpr int BOARD_TX                = 0;

constexpr int BOARD_PIN_MOTOR0        = 10;
constexpr int BOARD_PIN_MOTOR1        = 11;
constexpr int BOARD_PIN_MOTOR2        = 12;
constexpr int BOARD_PIN_MOTOR3        = 13;
constexpr uint8_t BOARD_MOTOR_PINS[4] = {BOARD_PIN_MOTOR0, BOARD_PIN_MOTOR1,
                                         BOARD_PIN_MOTOR2, BOARD_PIN_MOTOR3};

constexpr int BOARD_LED_PIN           = 25; // W 32

constexpr int BOARD_BATTERY_PIN       = 28; // GPIO28 ADC2
// constexpr int BOARD_ANALOG_3V3        = 15;

// constexpr int BOARD_BUZZER = 0;

// constexpr bool HAS_NEOPIXEL           = false;
// constexpr int BOARD_NEOPIXEL_PIN      = 0;

// constexpr Stream* BOARD_USB_SERIAL = &Serial;
// constexpr Stream* BOARD_ALT_SERIAL = &Serial1;