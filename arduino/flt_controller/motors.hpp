#include <Servo.h>

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
// constexpr int PIN_MOTOR1 = 0;
// constexpr int PIN_MOTOR2 = 0;
// constexpr int PIN_MOTOR3 = 0;

class QuadESC {
  public:
    QuadESC() : armed(false) {

    }

    ~QuadESC() {
        if (armed)
            stop();
    }

    void stop() {
        motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
        // motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
        // motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
        // motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
        motor0.detach();
        // motor1.detach();
        // motor2.detach();
        // motor3.detach();
    }

    // move to constructor?
    void init() {
        motor0.attach(PIN_MOTOR0, 1000, 2000);
        // motor1.attach(PIN_MOTOR1);
        // motor2.attach(PIN_MOTOR2);
        // motor3.attach(PIN_MOTOR3);
        // motor0.writeMicroseconds(MOTOR_ARM_START);
        // motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
        // motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
        // motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
        // delay(3000);
        // motor0.writeMicroseconds(0);
        // Serial.println("init done");
    }

    void arm() {
      motor0.writeMicroseconds(MOTOR_ARM_START);
      delay(2000);
      motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
      delay(1000);
    }

    void set(int m0, int m1, int m2, int m3) {
        motor0.writeMicroseconds(m0);
        // motor1.writeMicroseconds(m1);
        // motor2.writeMicroseconds(m2);
        // motor3.writeMicroseconds(m3);
    }

  protected:
    Servo motor0;
    Servo motor1;
    Servo motor2;
    Servo motor3;

    bool armed;
};

// void motor_ramp() {
//     // ramp up to max
//     for (int i=MOTOR_ZERO_LEVEL; i < MOTOR_MAX_LEVEL; i+=100){
//       int val = i;
//       motors.set(val,val,val,val);
//       delay(500);
//     }

//     // ramp down to min
//     for (int i=MOTOR_MAX_LEVEL; i > MOTOR_ZERO_LEVEL; i-=100){
//       int val = i;
//       motors.set(val,val,val,val);
//       delay(500);
//     }

//     // set to 0
//     int val = MOTOR_ZERO_LEVEL;
//     motors.set(val,val,val,val);
// }

int motor_limit(const int val){
    return val >= MOTOR_MAX_LEVEL ? MOTOR_MAX_LEVEL : val <= MOTOR_ZERO_LEVEL ? MOTOR_ZERO_LEVEL : val;
}