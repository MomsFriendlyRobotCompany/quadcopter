
#pragma once

#include <math.h>

// https://x-engineer.org/low-pass-filter/
// https://www.electronics-tutorials.ws/filter/filter_2.html
// phase shift = -atan(2*pi*f*rc)*180/pi
//   f: cutoff freq
//   rc: rc const
template<typename T>
class LPF {
  public:
  // LPF(T freq_cut, T dt) {
  //   T rc = 1.0 / (2.0 * M_PI * freq_cut);
  //   alpha = dt / (rc + dt);
  //   vlast = 0.0;
  // }

  LPF(T dt) {
    T freq_cut = 0.5 / dt; // fc = 1/2 * sample_rate
    T rc = 1.0 / (2.0 * M_PI * freq_cut);
    alpha = dt / (rc + dt);
    vlast = 0.0;
  }

  T update(T vin) {
    T out = alpha*vin + (1.0 - alpha)*vlast;
    vlast = out;
    return out;
  }

  protected:
  T alpha;
  T vlast;
};