/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2014 Kevin Walchko
* see LICENSE for full details
\**************************************************/
#pragma once

#include <Servo.h>
#include <quadcopter.hpp>

/**
BLHeli 15a ESC
Features from pdf manual
- Perfect for 1806, 2204, 2205, brushless motor in QAV180, QAV 210, QAV250 and
other small drone
- Works with 2S to 4S input ‐ Rated at 15A continuous and 25A burst (5s max)
- All N‐FET design with external oscillator for steady performance across
different thermal and voltage conditions

## Timings

Supports standard 1-2 msec RC servo timings at >= 50Hz

- Minimum: 920 - 1050 usec (or 0.920 - 1.05 msec)
- Middle: 1500 usec
- Max 1800 - 2000 usec

## Power ON Sequence

1. Turn power on: 3 short rising tone beeps
2. Set middle throttle: 1 long low tone beep
3. Set zero throttle: 1 long high tone beep
4. Motor is ready to command

*/


// ----- Motor PWM Levels -----
constexpr int MOTOR_MAX_LEVEL  = 2000;
constexpr int MOTOR_ZERO_LEVEL = 1000;
constexpr int MOTOR_ARM_START  = 1500;


class QuadESC {
public:
  QuadESC(): armed(false),
      motor0_val(MOTOR_ZERO_LEVEL),
      motor1_val(MOTOR_ZERO_LEVEL),
      motor2_val(MOTOR_ZERO_LEVEL),
      motor3_val(MOTOR_ZERO_LEVEL) {

    motor0.attach(BOARD_PIN_MOTOR0, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
    motor1.attach(BOARD_PIN_MOTOR1, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
    motor2.attach(BOARD_PIN_MOTOR2, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
    motor3.attach(BOARD_PIN_MOTOR3, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
  }

  ~QuadESC() {
    if (armed) disarm();

    motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
    motor0.detach();
    motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
    motor1.detach();
    motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
    motor2.detach();
    motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
    motor3.detach();
  }

  // Arms the ESC's and makes them ready for flight
  void arm() {
    motor0.writeMicroseconds(MOTOR_ARM_START);
    motor1.writeMicroseconds(MOTOR_ARM_START);
    motor2.writeMicroseconds(MOTOR_ARM_START);
    motor3.writeMicroseconds(MOTOR_ARM_START);

    delay(2000);

    motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
    motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
    motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
    motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);

    delay(1000);

    this->armed = true;
  }

  void disarm() {
    set(0,0,0,0);
    this->armed = false;
  }

  // Set the ESC for each motor to a PWM
  void set(const int m0, const int m1, const int m2, const int m3) {
    if (!armed) return;
    motor0_val = motor_limit(m0);
    motor1_val = motor_limit(m1);
    motor2_val = motor_limit(m2);
    motor3_val = motor_limit(m3);

    motor0.writeMicroseconds(motor0_val);
    motor1.writeMicroseconds(motor1_val);
    motor2.writeMicroseconds(motor2_val);
    motor3.writeMicroseconds(motor3_val);
  }

  inline
  void set(const quad::cmd_motors_t& cmd) {
    set(cmd.a,cmd.b,cmd.c,cmd.d);
  }

  const quad::motors_t get_msg() {
    quad::motors_t msg{0};
    msg.a    = motor0_val;
    msg.b    = motor1_val;
    msg.c    = motor2_val;
    msg.d    = motor3_val;
    msg.armed = (armed) ? 1 : 0;
    return msg;
  }

protected:
  Servo motor0;
  Servo motor1;
  Servo motor2;
  Servo motor3;

  int motor0_val;
  int motor1_val;
  int motor2_val;
  int motor3_val;

  bool armed;

  inline int motor_limit(const int val) {
    return constrain(val, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
  }
};





















  // bool init(const uint8_t pins[4]) {
  //   motor0.attach(pins[0], MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
  //   motor1.attach(pins[1], MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
  //   motor2.attach(pins[2], MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
  //   motor3.attach(pins[3], MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
  //   return true;
  // }

  // void stop() {
  //   if (!armed) return;

  //   bool state[4]{true, true, true, true};
  //   const float steps = 1.0 / 5.0;
  //   const int incrs[4]{
  //       int((motor0_val - MOTOR_ZERO_LEVEL) * steps),
  //       int((motor1_val - MOTOR_ZERO_LEVEL) * steps),
  //       int((motor2_val - MOTOR_ZERO_LEVEL) * steps),
  //       int((motor3_val - MOTOR_ZERO_LEVEL) * steps),
  //   };
  //   int vals[4];

  //   // int incr = (motor_val - MOTOR_ZERO_LEVEL) / 5.0;
  //   // Serial.println(incr);
  //   while (true) {
  //     int *motor_val = nullptr;
  //     for (int num = 0; num < 4; ++num) {
  //       switch (num) {
  //       case 0:
  //         motor_val = &motor0_val;
  //         break;
  //       case 1:
  //         motor_val = &motor1_val;
  //         break;
  //       case 2:
  //         motor_val = &motor2_val;
  //         break;
  //       case 3:
  //         motor_val = &motor3_val;
  //         break;
  //       }

  //       if (*motor_val > MOTOR_ZERO_LEVEL) {
  //         *motor_val = motor_limit(*motor_val - incrs[num]);
  //         vals[num]  = *motor_val;
  //       }
  //       else {
  //         vals[num]  = MOTOR_ZERO_LEVEL;
  //         state[num] = false;
  //       }
  //     }
  //     set(vals[0], vals[1], vals[2], vals[3]);
  //     // Serial.println(motor_val[0]);
  //     delay(100);

  //     if (state[0] == false && state[1] == false && state[2] == false &&
  //         state[3] == false)
  //       break;
  //   }
  // }


// #elif defined(linux) || defined(__APPLE__)

// class QuadESC : public Alarm {
// public:
//   QuadESC()
//       : armed(false), motor0_val(MOTOR_ZERO_LEVEL),
//         motor1_val(MOTOR_ZERO_LEVEL), motor2_val(MOTOR_ZERO_LEVEL),
//         motor3_val(MOTOR_ZERO_LEVEL) {}

//   ~QuadESC() {}

//   bool init(const uint8_t pins[4]) { return true; }

//   void stop() {}

//   /* Arms the ESC's and makes them ready for flight */
//   void arm() { armed = true; }

//   /* Set the ESC for each motor to a PWM */
//   void set(const int m0, const int m1, const int m2, const int m3) {}

//   void incr(const int delta) {
//     if (!armed) return;
//     motor0_val += delta;
//     motor1_val += delta;
//     motor2_val += delta;
//     motor3_val += delta;
//   }

//   void ramp() {}

//   motors4_t get_msg() {
//     msg.m0    = motor0_val;
//     msg.m1    = motor1_val;
//     msg.m2    = motor2_val;
//     msg.m3    = motor3_val;
//     msg.armed = (armed) ? 1 : 0;
//     return msg;
//   }

// protected:
//   motors4_t msg;

//   int motor0_val;
//   int motor1_val;
//   int motor2_val;
//   int motor3_val;

//   bool armed;

//   inline int motor_limit(const int val) {
//     return constrain(val, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
//   }
// };

// #endif