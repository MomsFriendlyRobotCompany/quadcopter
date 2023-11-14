

#include <stdio.h>
#include <cmath>

#include "picolib/picolib.hpp"

#include <gcisensors.hpp>

using namespace LIS3MDL;
using namespace gci::sensors;

constexpr uint i2c_port = 0;
constexpr uint i2c_scl  = I2C0_SCL_PIN;
constexpr uint i2c_sda  = I2C0_SDA_PIN;

int main() {
  stdio_init_all();
  wait_for_usb();

  TwoWire tw;
  uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C));
  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);

  gciLIS3MDL mag;
  mag.init_tw(i2c_port);
  while (true) {
    int err = mag.init(RANGE_4GAUSS, ODR_155HZ);
    if (err == 0) break;
    printf("*** mag error: %d ***\n", err);
    sleep_ms(1000);
  }

  while(1) {
    sleep_ms(10);
    bool ok = false;
    const mag_t m = mag.read_cal();
    if (m.ok) {
      printf("mag: %f %f %f\n", m.x, m.y, m.z);
    }
  }
}
