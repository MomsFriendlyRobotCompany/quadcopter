#pragma once

/**
Take a look at borderflight drivers:

lis3mdl: https://github.com/bolderflight/lis3mdl
bmp3: https://github.com/bolderflight/bmp3
lsm6dsox: None

Another:
lsm6ssox: https://github.com/arduino-libraries/Arduino_LSM6DSOX

*/

#include <stdint.h>
#include <cmath>
#include <msgids.hpp> // MsgIDs


namespace Units {
constexpr float rad2deg = 180.0f/M_PI;
constexpr float deg2rad = M_PI/180.0f;
constexpr float rps2rpm = 0.0f; // FIXME
constexpr float dps2rps = 0.0f; // FIXME


float f2c(float f){ return 0.5555556f * (f - 32.0f); }
float c2f(float c){ return 1.8 * c + 32; }

}

namespace Nav {
constexpr float earth_semi_major = 6;  
}

template<uint32_t size> // number of floats
struct Buffer {
  union {
    float    f[size];   // float
    uint32_t l[size];   // long
    uint16_t s[size*2]; // short
    uint8_t  b[size*4]; // byte
  };

  void clear() {
    memset(b, 0, size);
  }
};

template<uint32_t flsz> // number of floats
class Sensor {
  public:
    Sensor(uint8_t Id): id(Id), bsize(flsz * 4), found(false) {}
    
    bool found;
    Buffer<flsz> data;
    const uint16_t bsize; // length of array
    const uint8_t id;
  // protected:
};