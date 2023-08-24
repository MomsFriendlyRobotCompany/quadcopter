#include <gecko2.hpp>

// quadcopter system
System q;

void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(5);
  while (!Serial)
    delay(10);

  // for heartbeat
  uint16_t sensors = heartbeat_t::ACCEL;
  sensors |= heartbeat_t::GYRO;
  sensors |= heartbeat_t::BAROMETER;
  sensors |= heartbeat_t::TEMPERATURE;
  sensors |= heartbeat_t::RANGE;
  sensors |= heartbeat_t::BATTERY;
  sensors |= heartbeat_t::COMPASS;
  bool ready = q.init(sensors, Serial);

}

void loop() {
  q.loop();
}
