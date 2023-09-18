
#pragma once

#if defined(ARDUINO)
  #if defined(ARDUINO_ITSYBITSY_M0)
    #include "boards/Adafruit_ItsyBitsy_M0.hpp"
  #elif defined(ARDUINO_ITSYBITSY_M4)
    #include "boards/Adafruit_ItsyBitsy_M4.hpp"
  // #elif defined(ARDUINO_ARCH_RP2040) \
  //       || defined(ARDUINO_RASPBERRY_PI_PICO) \
  //       || defined(ARDUINO_ADAFRUIT_ITSYBITSY_RP2040) \
  //       || defined(ARDUINO_ADAFRUIT_QTPY_RP2040)
  //   #include "boards/generic.hpp"
  // #else
  //   #include "boards/generic.hpp"
  #endif
#endif

#include "quad_msgs.hpp"
#include "updated.hpp"

