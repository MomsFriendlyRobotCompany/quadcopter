#pragma once

#include <Servo.h>
// use default ardiuno servo for arm
// or use servoesp32 for an esp32 board

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

// ----- Motor locations -----
constexpr int PIN_MOTOR0 = 3;

/*
For testing, this only does ONE motor at a time. 
*/
class QuadESC {
  public:
    QuadESC() : armed(false), motor_val(MOTOR_ZERO_LEVEL) {}

    ~QuadESC() {
        if (armed)
            stop();
        
        motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
        motor0.detach();
    }

    void stop() {
      int incr = (motor_val - MOTOR_ZERO_LEVEL) / 5.0;
      // Serial.println(incr);
      while (motor_val > MOTOR_ZERO_LEVEL){
        motor_val = motor_limit(motor_val - incr);
        set(motor_val);
        // Serial.println(motor_val[0]);
        delay(100);
      }
    }

    // move to constructor?
    void init() {
        motor0.attach(PIN_MOTOR0, 1000, 2000);
    }

    void arm() {
      motor0.writeMicroseconds(MOTOR_ARM_START);
      delay(2000);
      motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
      delay(1000);
    }

    void set(int m0) {
        m0 = motor_limit(m0);
        motor0.writeMicroseconds(m0);
    }

    int incr(int delta) {
        motor_val += delta;
        set(motor_val);
        return motor_val;
    }

  protected:

    inline int motor_limit(const int val){
        // return val >= MOTOR_MAX_LEVEL ? MOTOR_MAX_LEVEL : val <= MOTOR_ZERO_LEVEL ? MOTOR_ZERO_LEVEL : val;
        return constrain(val, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
    }

    Servo motor0;
    int motor_val;
    bool armed;
};


// int motor_limit(const int val){
//     return val >= MOTOR_MAX_LEVEL ? MOTOR_MAX_LEVEL : val <= MOTOR_ZERO_LEVEL ? MOTOR_ZERO_LEVEL : val;
// }