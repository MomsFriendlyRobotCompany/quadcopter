
#pragma once

#include "memory.hpp"
#include "picolib/picolib.hpp"

#include <mavlink.h>

#include <gcigps.hpp>
#include <gciSensors.hpp>
#include <messages.hpp>
#include <squaternion.hpp>
#include <yivo.hpp>

using namespace LSM6DSOX;
using namespace BMP390;
using namespace LIS3MDL;
using namespace gci::sensors;

// delared in main.cpp -------
extern gci::GPS gps;
extern gciLIS3MDL mag;
extern gciLSM6DSOX imu;
extern gciBMP390 bmp;
extern Yivo yivo;
extern Serial Serial0;
extern Serial Serial1;
extern ADC adc;
//----------------------------

void handle_ins() {
  bool ok = false;
  const LIS3MDL::mag_t m = mag.read_cal();
  if (m.ok) {
    printf("mag: %f %f %f\n", m.x, m.y, m.z);
    memory.sensors += STATUS_MAGS;
  }

  sox_t i = imu.read();
  if (i.ok) {
    printf("accel: %f %f %f  gyro: %f %f %f\n", i.a.x, i.a.y, i.a.z, i.g.x,
          i.g.y, i.g.z);

    memory.sensors += (STATUS_ACCELS|STATUS_GYROS);
  }

  // memory.status = SET_BITS(memory.status, STATUS_ACCELS|STATUS_GYROS|STATUS_MAGS);
  // memory.sensors += (STATUS_ACCELS|STATUS_GYROS|STATUS_MAGS);
}

void handle_pt() {
  bool ok = false;
  pt_t pt = bmp.read();
  if (pt.ok) {
    printf("press: %f   temp: %f\n", pt.press, pt.temp);
    memory.sensors += (STATUS_PRESS|STATUS_TEMP);
  }
  // memory.status = SET_BITS(memory.status, STATUS_PRESS|STATUS_TEMP);
}

void handle_battery() {
  float batt = adc.read(ADC_BATT_PIN);
  // printf("battery: %f\n", batt);
  memory.sensors += STATUS_BATTERY;
}

void handle_gps() {
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
      if (ok) {
        printf("GGA lat: %f lon: %f\n", gga.lat, gga.lon);
        memory.sensors += STATUS_GPS;
      }
      else printf("Bad parsing\n");
    }
  }

  // memory.status = SET_BITS(memory.status, STATUS_GPS);
  // memory.timer1hz = false;
}