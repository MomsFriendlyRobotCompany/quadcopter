

#include <cmath>
#include <stdio.h>

#include "picolib/picolib.hpp"

#include <gcisensors.hpp>

using namespace LSM6DSOX;
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

  gciLSM6DSOX imu;
  imu.init_tw(i2c_port);
  while (true) {
    uint8_t err = imu.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ);
    if (err == 0) break;
    printf("*** imu error: %d ***\n", err);
    sleep_ms(1000);
  }

  /// Double check SIGNS on bias + or - ///
  float sm[12]{1.00268927,  -0.00056029, -0.00190925, -0.00492348,
               -0.00138898, 0.99580818,  -0.00227335, 0.00503835,
               -0.01438271, 0.00673172,  0.9998954,   -0.01364759};
  imu.set_acal(sm);

  float gm[12]{1.0f, 0,           0, -0.00889949, 0,    1.0f,
               0,    -0.00235061, 0, 0,           1.0f, -0.00475294};
  imu.set_gcal(gm);

  while (1) {
    sleep_ms(10);
    sox_t i = imu.read();
    if (i.ok == false) {
      printf("** bad read **\n");
      continue;
    }

    printf("accel: %f %f %f  gyro: %f %f %f\n", i.a.x, i.a.y, i.a.z, i.g.x,
           i.g.y, i.g.z);
  }
}
