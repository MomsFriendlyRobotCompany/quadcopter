
#pragma once

#include "memory.hpp"
#include "picolib/picolib.hpp"

// #include <mavlink.h>
#include <cstring> // memset
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
  imu_agmpt_t msg;
  memset(&msg, 0, sizeof(msg));

  lis3mdl_t m = mag.read_cal();
  if (m.ok) {
    msg.m.x = m.x;
    msg.m.y = m.y;
    msg.m.z = m.z;

    memory.mags = m;
    // printf("mag: %f %f %f\n", m.x, m.y, m.z);
    memory.sensors += STATUS_MAGS;
  }

  lsm6dsox_t i = imu.read();
  if (i.ok) {
    msg.a.x = i.a.x;
    msg.a.y = i.a.y;
    msg.a.z = i.a.z;
    msg.g.x = i.g.x;
    msg.g.y = i.g.y;
    msg.g.z = i.g.z;

    memory.imu = i;
    // printf("accel: %f %f %f  gyro: %f %f %f\n",
    //   i.a.x, i.a.y, i.a.z,
    //   i.g.x, i.g.y, i.g.z);

    memory.sensors += STATUS_AG;
  }

  bmp390_t pt = bmp.read();
  if (pt.ok) {
    msg.pressure = pt.press;
    msg.temperature = pt.temp;

    memory.press_temp = pt;
    // printf("press: %f   temp: %f\n", pt.press, pt.temp);
    memory.sensors += STATUS_PT;
  }

  msg.timestamp = time_since_boot_ms();

  yivopkt_t y;
  y.pack(MSG_IMU, (uint8_t*)&msg, sizeof(msg));
  write_stdout(y.data(), y.size());
}

void run_nav_filter() {}

void handle_battery() {
  float batt = adc.read(ADC_BATT_PIN);
  // printf("battery: %f\n", batt);
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
    // printf("GGA lat: %f lon: %f\n", gga.lat, gga.lon);
    memory.sensors += STATUS_GPS;

    satnav_t msg;
    msg.lat = gga.lat;
    msg.lon = gga.lon;
    msg.alt = 0.0f;
    yivopkt_t y;
    y.pack(MSG_GPS, (uint8_t*)&msg, sizeof(msg));
    write_stdout(y.data(), y.size());
  }
}

void handle_health() {
  // send heartbeat msg
  // printf("+ Health +\n");
  heartbeat_t msg {time_since_boot_ms(), 0};
  yivopkt_t y;
  y.pack(MSG_HEARTBEAT, (uint8_t*)&msg, sizeof(msg));
  write_stdout(y.data(), y.size());
}