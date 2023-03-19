#pragma once

#include <gciSensors.hpp>
#include <messages.hpp>

class Heartbeat: public Alarm {
  public:
  Heartbeat(const uint32_t msec): Alarm(msec) {
    msg.id = 1;
    msg.sensors = Heartbeat_t::ACCEL;
    msg.sensors |= Heartbeat_t::GYRO;
    msg.sensors |= Heartbeat_t::BAROMETER;
    msg.sensors |= Heartbeat_t::TEMPERATURE;
    msg.sensors |= Heartbeat_t::RANGE;
    msg.sensors |= Heartbeat_t::BATTERY;
    msg.sensors |= Heartbeat_t::COMPASS;
  }

  const Heartbeat_t get_msg() {
    msg.timestamp = millis();
    return msg;
  }

  protected:
  Heartbeat_t msg;
};