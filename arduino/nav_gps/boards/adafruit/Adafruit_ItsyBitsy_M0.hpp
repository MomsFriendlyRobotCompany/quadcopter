#pragma once

constexpr int BOARD_SDA               = 1;
constexpr int BOARD_SCL               = 1;
constexpr int BOARD_RX                = 1;
constexpr int BOARD_TX                = 1;

constexpr int BOARD_PIN_MOTOR0        = 7;
constexpr int BOARD_PIN_MOTOR1        = 9;
constexpr int BOARD_PIN_MOTOR2        = 10;
constexpr int BOARD_PIN_MOTOR3        = 11;
constexpr uint8_t BOARD_MOTOR_PINS[4] = {BOARD_PIN_MOTOR0, BOARD_PIN_MOTOR1,
                                         BOARD_PIN_MOTOR2, BOARD_PIN_MOTOR3};

constexpr int BOARD_LED_PIN           = 13;

constexpr int BOARD_ANALOG_BATTERY    = 15;
constexpr int BOARD_ANALOG_3V3        = 15;

constexpr bool HAS_NEOPIXEL           = false;
constexpr int BOARD_NEOPIXEL_PIN      = 18;
