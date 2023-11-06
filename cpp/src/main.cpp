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
// #include "gps.hpp"
#include "led.hpp"
#include "pwm.hpp"
#include "memory.hpp"

#include <gcigps.hpp>
#include <gciSensors.hpp>
#include <messages.hpp>
#include <squaternion.hpp>
#include <yivo.hpp>

#include <mavlink.h>

using namespace std;
using namespace LSM6DSOX;
using namespace BMP390;
using namespace LIS3MDL;
using namespace gci::sensors;

gci::GPS gps;
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

bool callback_100hz(struct repeating_timer *t) {
  memory.timer100hz.set();
  return true;
}


bool callback_10hz(struct repeating_timer *t) {
  memory.timer10hz.set();
  return true;
}

bool callback_1hz(struct repeating_timer *t) {
  memory.timer1hz.set();
  return true;
}

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

  Serial0.init(115200, 0, UART0_TX_PIN, UART0_RX_PIN);

  imu.init_tw(i2c_port);
  while (true) {
    uint8_t err = imu.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ);
    if (err == 0) break;
    printf("*** imu error: %d ***\n", err);
    sleep_ms(1000);
  }

  bmp.init_tw(i2c_port);
  while (true) {
    int err = bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X);
    if (err == 0) break;
    printf("*** bmp error: %d ***\n", err);
    sleep_ms(1000);
  }

  mag.init_tw(i2c_port);
  while (true) {
    int err = mag.init(RANGE_4GAUSS, ODR_155HZ);
    if (err == 0) break;
    printf("*** mag error: %d ***\n", err);
    sleep_ms(1000);
  }

  // Blink LED /////////////////////
  led_init();

  // GPS ///////////////////////////
  Serial1.init(9600, 1, UART1_TX_PIN, UART1_RX_PIN);
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

  struct repeating_timer timer_100hz;
  add_repeating_timer_ms(-10, callback_100hz, NULL, &timer_100hz);
  struct repeating_timer timer_1hz;
  add_repeating_timer_ms(-1000, callback_1hz, NULL, &timer_1hz);

  sleep_ms(100);


  mavlink_message_t message;

  const uint8_t system_id = 42;
  const uint8_t base_mode = 0;
  const uint8_t custom_mode = 0;
  mavlink_msg_heartbeat_pack_chan(
    system_id,
    MAV_COMP_ID_PERIPHERAL,
    MAVLINK_COMM_0,
    &message,
    MAV_TYPE_GENERIC,
    MAV_AUTOPILOT_GENERIC,
    base_mode,
    custom_mode,
    MAV_STATE_STANDBY);

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const int len = mavlink_msg_to_send_buffer(buffer, &message);

  while (1) {
    if (memory.timer100hz == true) {
      const LIS3MDL::mag_t m = mag.read_cal();
      if (m.ok) {
        printf("mag: %f %f %f\n", m.x, m.y, m.z);
      }

      sox_t i = imu.read();
      if (i.ok) {
        printf("accel: %f %f %f  gyro: %f %f %f\n", i.a.x, i.a.y, i.a.z, i.g.x,
              i.g.y, i.g.z);
      }

      memory.status = SET_BITS(memory.status, STATUS_ACCELS|STATUS_GYROS|STATUS_MAGS);
      // memory.timer100hz = false;
    }

    if (memory.timer1hz == true) {
      pt_t pt = bmp.read();
      if (pt.ok) {
        printf("press: %f   temp: %f\n", pt.press, pt.temp);
      }

      bool ok = false;
      while (Serial1.available()) {
        char c = (char)Serial1.read();
        ok = gps.read(c);
        if (ok) break;
      }

      if (ok) {
        gga_t gga;
        gci::GpsID id = gps.get_id();
        if (id == gci::GpsID::GGA) {
          ok = gps.get_msg(gga);
          if (ok) printf("GGA lat: %f lon: %f\n", gga.lat, gga.lon);
          else printf("Bad parsing\n");
        }
      }

      float batt = adc.read(ADC_BATT_PIN);
      // printf("battery: %f\n", batt);

      memory.status = SET_BITS(memory.status, STATUS_PRESS|STATUS_TEMP|STATUS_GPS|STATUS_BATTERY);
      // memory.timer1hz = false;
    }

    watchdog_update();
  }
}