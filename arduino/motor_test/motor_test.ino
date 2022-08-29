
// #include "debug.hpp"
// #include "imu.hpp"
#include "motors.hpp"
// #include "packer.hpp"
// #include <Wire.h>
#include <stdint.h>

// using namespace std;

// gciLSOXLIS imu;
// gciDPS310 press;
QuadESC motors;


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

/* Toggle board's LED on/off */
class BlinkLED {
  public:
  BlinkLED(const uint32_t delaytime): blink_time(millis()), led_blink(true), dt(delaytime) {}

  void update() {
    uint32_t now = millis();
    if (now > blink_time){
      led(led_blink);
      led_blink = !led_blink;
      blink_time = now + dt;
    }
  }

  protected:

  void led(bool val) {
      constexpr int LED_PIN = 13;
      constexpr int wait_time = 500;

      if (val) digitalWrite(LED_PIN, HIGH);
      else digitalWrite(LED_PIN, LOW);
  }

  const uint32_t dt;
  uint32_t blink_time;
  bool led_blink;
};


BlinkLED blinker(500);

void setup() {

    Serial.begin(115200);

    motors.init();
    motors.arm();
    Serial.println("Motors ready and armed");
    
}

uint32_t blink_time = 0;
bool led_blink = true;
// int motor_val[4] = {1000};
constexpr int incr = 10;
bool start = true;

void loop() {

    if (start) {
      start = false;
      motors.set(1050);
    }

    // serial ascii input
    if (Serial.available() > 0) {
        int inByte = Serial.read();
        
        if (inByte == 'a') { // add
            int val = motors.incr(incr);
            Serial.println(val);
        }
        else if (inByte == 'd') { // decrease
            int val = motors.incr(-incr);
            Serial.println(val);
        }
        else if (inByte == 's') { // stop
            motors.stop();
        }
    }

    blinker.update();
}
