#pragma once


class Hertz {
  public:
  Hertz(const uint32_t delaytime): blink_time(millis()), dt(delaytime) {}

  protected:
  bool check(){
    uint32_t now = millis();
    if (now > blink_time){
      // led(led_blink);
      // led_blink = !led_blink;
      blink_time = now + dt;
      return true;
    }
    return false;
  }

  const uint32_t dt;
  uint32_t blink_time;
};


/* Toggle board's LED on/off */
class BlinkLED: public Hertz {
  public:
  // BlinkLED(const uint32_t delaytime): blink_time(millis()), led_blink(true), dt(delaytime) {}
  BlinkLED(const uint32_t delaytime): led_blink(true), Hertz(delaytime) {}

  void update() {
    // uint32_t now = millis();
    // if (now > blink_time){
    //   led(led_blink);
    //   led_blink = !led_blink;
    //   blink_time = now + dt;
    // }
    if (check()) {
      led(led_blink);
      led_blink = !led_blink;
    }
  }

  protected:

  void led(bool val) {
      constexpr int LED_PIN = 13;
      constexpr int wait_time = 500;

      if (val) digitalWrite(LED_PIN, HIGH);
      else digitalWrite(LED_PIN, LOW);
  }

  // const uint32_t dt;
  // uint32_t blink_time;
  bool led_blink;
};


// class Telemetry: public Hertz {
//   public:
//   Telemetry(const uint32_t delaytime): Hertz(delaytime) {}

//   void update(int a, int b, int c, int d) {
//     if (check()){
//       packetMotor(a,b,c,d);
//     }
//   }
// };