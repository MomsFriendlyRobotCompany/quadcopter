

#include <cmath>
#include <stdio.h>

#include <gcisensors.hpp>

#include "picolib/picolib.hpp"

using namespace BMP390;
using namespace gci::sensors;

constexpr uint i2c_port = 0;
constexpr uint i2c_scl  = I2C0_SCL_PIN;
constexpr uint i2c_sda  = I2C0_SDA_PIN;

float altitude(const float p) {
  // Probably best not to run here ... very computational.
  // pre compute some of this?
  // call atmospalt() ... like matlab?
  // same as mean sea level (MSL) altitude
  // Altitude from pressure:
  // https://www.mide.com/air-pressure-at-altitude-calculator
  // const float Tb = 15; // temperature at sea level [C] - doesn't work
  // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
  constexpr float Tb    = 288.15f;           // temperature at sea level [K]
  constexpr float Lb    = -0.0065f;          // lapse rate [K/m]
  constexpr float Pb    = 101325.0f;         // pressure at sea level [Pa]
  constexpr float R     = 8.31446261815324f; // universal gas const [Nm/(mol K)]
  constexpr float M     = 0.0289644f; // molar mass of Earth's air [kg/mol]
  constexpr float g0    = 9.80665f;   // gravitational const [m/s^2]

  constexpr float exp   = -R * Lb / (g0 * M);
  constexpr float scale = Tb / Lb;
  constexpr float inv_Pb = 1.0f / Pb;

  return scale * (pow(p * inv_Pb, exp) - 1.0);
}

int main() {
  stdio_init_all();
  wait_for_usb();

  TwoWire tw;
  uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C));
  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);

  gciBMP390 bmp;
  bmp.init_tw(i2c_port);
  while (true) {
    int err = bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X); // 25 hz
    if (err == 0) break;
    printf("*** bmp error: %d ***\n", err);
    sleep_ms(1000);
  }

  uint32_t cnt   = 1;
  uint32_t epoch = time_since_boot_ms();
  float hz       = 0.0f;

  while (1) {
    if (cnt++ % 100 == 0) {
      uint32_t now = time_since_boot_ms();
      float dt     = (float)(now - epoch) / 1000.0f;
      hz           = 100.0f / dt;
      epoch        = now;
      cnt          = 1;
    }
    sleep_ms(40); // 25 hz

    bool ok = false;
    pt_t pt = bmp.read();
    if (pt.ok == false) {
      printf("** bad reading **\n");
      continue;
    }

    float alt = altitude(pt.press);
    printf("hz: %f   press: %f   temp: %f    alt: %f\n", hz, pt.press, pt.temp,
           alt);
  }
}