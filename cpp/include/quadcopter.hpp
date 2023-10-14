
#pragma once

// https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html
// https://github.com/raspberrypi/pico-sdk/blob/master/src/boards/include/boards/pico.h

#if defined(ARDUINO)
  #if defined(ARDUINO_ITSYBITSY_M0)
    #include "boards/Adafruit_ItsyBitsy_M0.hpp"
  #elif defined(ARDUINO_ITSYBITSY_M4)
    #include "boards/Adafruit_ItsyBitsy_M4.hpp"
  // #elif defined(ARDUINO_ARCH_RP2040) \
  //       || defined(ARDUINO_RASPBERRY_PI_PICO) \
  //       || defined(ARDUINO_ADAFRUIT_ITSYBITSY_RP2040) \
  //       || defined(ARDUINO_ADAFRUIT_QTPY_RP2040)
  // #elif defined(ARDUINO_RASPBERRY_PI_PICO)
  #elif defined(RASPBERRYPI_PICO)
    #include "boards/Pi_Pico.hpp"
  // #else
  //   #include "boards/generic.hpp"
  #endif
#endif

// struct kvec_t {
//   float x,y,z;
//   float operator[](size_t i) {
//     return (i == 0) ? x : (i == 1) ? y : z;
//   }
// };

#include "quad_msgs.hpp"
#include "updated.hpp"
#include "filter.hpp"

