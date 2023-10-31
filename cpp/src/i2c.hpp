
#pragma once

// // #include "pico/stdlib.h"
// #include "hardware/i2c.h"

// constexpr uint I2C0_SDA_PIN = 8;
// constexpr uint I2C0_SCL_PIN = 9;
// constexpr uint I2C1_SDA_PIN = 14;
// constexpr uint I2C1_SCL_PIN = 15;
// constexpr uint I2C_100KHZ = 100 * 1000;
// constexpr uint I2C_400KHZ = 400 * 1000;
// constexpr bool I2C_HOLD_BUS = true;
// constexpr bool I2C_RELEASE_BUS = false;

// // template<uint8_t port,uint8_t pin_sda, uint8_t pin_scl>
// class TwoWire {
//   uint8_t addr;
//   i2c_inst_t* i2c;

//   public:
//   TwoWire(): addr(0) {}
//   ~TwoWire() {}

//   // Either i2c0 or i2c1
//   // void init(uint baud) {
//   void init(uint baud, uint8_t port, uint8_t pin_sda, uint8_t pin_scl) {
//     if (port == 0) i2c = i2c0;
//     else if (port == 1) i2c = i2c1;
//     // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5
//     on a Pico) i2c_init(i2c, baud); gpio_set_function(pin_sda,
//     GPIO_FUNC_I2C); gpio_set_function(pin_scl, GPIO_FUNC_I2C);
//     gpio_pull_up(pin_sda);
//     gpio_pull_up(pin_scl);
//     // Make the I2C pins available to picotool
//     // bi_decl(
//     //   bi_2pins_with_func(
//     //     PICO_DEFAULT_I2C_SDA_PIN,
//     //     PICO_DEFAULT_I2C_SCL_PIN,
//     //     GPIO_FUNC_I2C
//     // ));
//   }

//   inline
//   void beginTransmission(uint8_t address) { addr = address; }

//   inline
//   void requestFrom(uint8_t address) { addr = address; }

//   inline
//   void endTransmission() {}

//   void write(const uint8_t reg, const uint8_t data) {
//     uint8_t out[2]{reg, data};
//     i2c_write_blocking(i2c, addr, out, 2, I2C_RELEASE_BUS);
//   }

//   bool read(const uint8_t reg, const uint8_t data_size, uint8_t *const data)
//   {
//     i2c_write_blocking(i2c, addr, &reg, 1, I2C_HOLD_BUS);
//     int ret = i2c_read_blocking(i2c, addr, data, data_size, I2C_RELEASE_BUS);
//     if (ret < 0) return false;
//     return true;
//   }

//   inline
//   size_t available() {
//     return i2c_get_read_available(i2c);
//   }

// };
