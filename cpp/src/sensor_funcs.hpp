
#pragma once

#include "memory.hpp"
#include "picolib/picolib.hpp"

// #include <mavlink.h>

#include <gciSensors.hpp>
#include <gcigps.hpp>
// #include <messages.hpp>
// #include <squaternion.hpp>
#include "messaging/yivo_comm.hpp"

using namespace LSM6DSOX;
using namespace BMP390;
using namespace LIS3MDL;
// using namespace gci::sensors;

// delared in main.cpp -------
extern gci::GPS gps;
extern gciLIS3MDL mag;
extern gciLSM6DSOX imu;
extern gciBMP390 bmp;
extern Serial Serial0;
extern Serial Serial1;
extern ADC adc;

extern SharedMemory_t memory;
extern Mutex sm_mutex;
//----------------------------

void handle_ins_sensors() {
  // bool ok                = false;
  mag_t m = mag.read_cal();
  if (m.ok) {
    memory.mags = m;
    printf("mag: %f %f %f\n", m.x, m.y, m.z);
    memory.sensors += STATUS_MAGS;
  }

  sox_t i = imu.read();
  if (i.ok) {
    memory.imu = i;
    printf("accel: %f %f %f  gyro: %f %f %f\n",
      i.a.x, i.a.y, i.a.z,
      i.g.x, i.g.y, i.g.z);

    memory.sensors += STATUS_AG;
  }

  pt_t pt = bmp.read();
  if (pt.ok) {
    memory.press_temp = pt;
    printf("press: %f   temp: %f\n", pt.press, pt.temp);
    memory.sensors += STATUS_PT;
  }
}

void run_nav_filter() {}

void handle_pt() {
  bool ok = false;
  pt_t pt = bmp.read();
  if (pt.ok) {
    memory.press_temp = pt;
    printf("press: %f   temp: %f\n", pt.press, pt.temp);
    memory.sensors += STATUS_PT;
  }
}

void handle_battery() {
  float batt = adc.read(ADC_BATT_PIN);
  printf("battery: %f\n", batt);
  memory.sensors += STATUS_BATTERY;
}

void handle_gps() {
  bool ok = false;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    ok     = gps.read(c);
    if (ok) break;
  }

  if (ok == false) return;

  gga_t gga;
  gci::GpsID id = gps.get_id();
  if (id == gci::GpsID::GGA) {
    ok = gps.get_msg(gga);
    if (ok == false) return;

    memory.gps = gga;
    printf("GGA lat: %f lon: %f\n", gga.lat, gga.lon);
    memory.sensors += STATUS_GPS;
  }
}

void handle_health() {
  // send heartbeat msg
  printf("+ Health +\n");
}