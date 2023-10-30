/*
enum gpio_function {
    GPIO_FUNC_XIP = 0,
    GPIO_FUNC_SPI = 1,
    GPIO_FUNC_UART = 2,
    GPIO_FUNC_I2C = 3,
    GPIO_FUNC_PWM = 4,
    GPIO_FUNC_SIO = 5,
    GPIO_FUNC_PIO0 = 6,
    GPIO_FUNC_PIO1 = 7,
    GPIO_FUNC_GPCK = 8,
    GPIO_FUNC_USB = 9,
    GPIO_FUNC_NULL = 0xf,
};
*/

#include <stdio.h>

#include "hardware/watchdog.h"
#include "tusb.h" // wait for USB
#include "defs.hpp"
#include "gps.hpp"
#include "pwm.hpp"
#include "led.hpp"

#include <messages.hpp>
#include <gciSensors.hpp>
#include <yivo.hpp>
#include <squaternion.hpp>

using namespace std;
using namespace LSM6DSOX;
using namespace BMP390;
using namespace LIS3MDL;
using namespace gci::sensors;

gciLIS3MDL mag;
gciLSM6DSOX imu;
gciBMP390 bmp;
Quaternion q;
Yivo yivo;

TwoWire tw;
Serial Serial0;
Serial Serial1;
ADC adc;

Servo m0;
Servo m1;
Servo m2;
Servo m3;

int main() {
  stdio_init_all();

  // wait for USB serial connection
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  if (watchdog_caused_reboot()) {
    printf("*** Watchdog Rebooted ***\n");
  }

  watchdog_enable(WATCHDOG_RESET, true);

  uint speed = tw.init(i2c_port, I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u buad: %u\n", i2c_port, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // ADC battery;
  bool ok = adc.init(ADC_BATT_PIN);
  if (ok == false) printf("*** Error ADC battery ***\n");

  Serial0.init(115200,0,UART0_TX_PIN,UART0_RX_PIN);

  imu.init_tw(i2c_port);
  while (true) {
    uint8_t err = imu.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (err == 0) break;
    puts("imu error");
    sleep_ms(1000);
  }

  bmp.init_tw(i2c_port);
  while (true) {
    int err = bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X);
    if (err == 0) break;
    puts("bmp error");
    sleep_ms(1000);
  }

  mag.init_tw(i2c_port);
  while (true) {
    int err = mag.init(RANGE_4GAUSS,ODR_155HZ);
    if (err == 0) break;
    puts("mag error");
    sleep_ms(1000);
  }

  // Blink LED /////////////////////
  led_init();

  // GPS ///////////////////////////
  Serial1.init(9600,1,UART1_TX_PIN,UART1_RX_PIN);
  Serial1.write(GCI_GGA, sizeof(GCI_GGA));
  Serial1.write(GCI_UPDATE_1HZ, sizeof(GCI_UPDATE_1HZ));
  Serial1.write(GCI_NOANTENNA, sizeof(GCI_NOANTENNA));
  // Serial1.write(GCI_BAUD_115200, sizeof(GCI_BAUD_115200));
  // uint baud = Serial1.set_baud(115200);
  // printf("/-- UART is reset to %u baud --/\n", baud);


  // Servo /////////////////////////
  m0.init(pwm_m0);
  m1.init(pwm_m1);
  m2.init(pwm_m2);
  m3.init(pwm_m3);

  sleep_ms(100);

  while (1) {
    float batt = adc.read(ADC_BATT_PIN);
    // printf("battery: %f\n", batt);

    pt_t pt = bmp.read();
    if (pt.ok) {
      printf("press: %f   temp: %f\n", pt.press, pt.temp);
    }

    const LIS3MDL::mag_t m = mag.read_cal();
    if (m.ok) {
      printf("mag: %f %f %f\n", m.x, m.y, m.z);
    }

    sox_t i = imu.read();
    if (i.ok) {
      printf("accel: %f %f %f  gyro: %f %f %f\n", i.a.x, i.a.y, i.a.z, i.g.x, i.g.y, i.g.z);
    }

    getGps(&Serial1);

    watchdog_update();

  }
}