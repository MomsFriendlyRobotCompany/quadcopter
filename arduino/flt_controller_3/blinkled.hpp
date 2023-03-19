#pragma once

#include <gciSensors.hpp>
#include "board.h"
// #include "Adafruit_ItsyBitsy_M0.hpp"


// constexpr int BOARD_LED_PIN = 13;
constexpr int wait_time = 500;

// Toggle board's LED on/off
class BlinkLED: public Alarm {
  public:
  BlinkLED(const uint32_t delaytime): led_blink(true), Alarm(delaytime) {}

  void update() {
    if (check()) {
      led(led_blink);
      led_blink = !led_blink;
    }
  }

  protected:

  void led(bool val) {
      if (val) digitalWrite(BOARD_LED_PIN, HIGH);
      else digitalWrite(BOARD_LED_PIN, LOW);
  }

  bool led_blink;
};


//////////////////////////////////////////

// class Hertz {
//   public:
//   Hertz(const uint32_t delaytime): mark(millis()), dt(delaytime) {}

//   bool check(){
//     uint32_t now = millis();
//     if (now > mark){
//       mark = now + dt;
//       return true;
//     }
//     return false;
//   }

//   protected:
//   const uint32_t dt;
//   uint32_t mark;
// };
