#define IMU_USE_UNCALIBRATED_DATA 0

#include <gecko2.hpp>
#include <yivo.hpp>
#include <gciSensors.hpp>
#include "sensor_suite.hpp"
#include "controller.hpp"

// quadcopter system
// System q;
Yivo yivo;
SensorSuite ss;
Controller cntl;

// imu_full_t imu; // imu state info

void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(100);
  while (!Serial)
    delay(10);

  Wire.begin();
  Wire.setClock(400000);

  Serial1.begin(TFmini::DEFAULT_BAUDRATE);

  while (!ss.init()){
    delay(1000);
    Serial.println("sensor suite error");
  }
  // ss.set(Serial);
  // ss.set(yivo);

  // cntl.init(BOARD_MOTOR_PINS);

  // // for heartbeat
  // uint16_t sensors = heartbeat_t::ACCEL;
  // sensors |= heartbeat_t::GYRO;
  // sensors |= heartbeat_t::BAROMETER;
  // sensors |= heartbeat_t::TEMPERATURE;
  // sensors |= heartbeat_t::RANGE;
  // sensors |= heartbeat_t::BATTERY;
  // sensors |= heartbeat_t::COMPASS;
  // bool ready = q.init(sensors, BOARD_MOTOR_PINS, Serial);

  // while (!ready) {
  //   Serial.println("Error initializing System"); // change to yivo message?
  //   delay(1000);
  // }
}

void loop() {
  // q.loop();
  bool update = ss.check();
  if (update) {
    // imu = ss.imu;
    yivo.pack(MSG_IMU_FULL, reinterpret_cast<uint8_t *>(&ss.imu), sizeof(imu_full_t));
    Serial.write(yivo.get_buffer(), yivo.get_total_size());
  }
  // Serial.println(".");
  // delay(1000);
}
