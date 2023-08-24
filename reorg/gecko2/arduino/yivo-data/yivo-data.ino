
#include <gecko2.hpp>
#include <yivo.hpp>
#include <gciSensors.hpp>
#include "sensor_suite.hpp"
#include "controller.hpp"


Yivo yivo;
SensorSuite ss;
Controller cntl;
QuadESC motors;
Toggle streamTelemetry;
MsgParser msgparser;
Performance perf;

// https://github.com/adafruit/ArduinoCore-samd/blob/master/cores/arduino/Tone.cpp
constexpr int tonePin = 10;
constexpr int interruptPin = 11;
constexpr uint8_t Frequency = 100; // Hz
volatile bool val = false;

void isr() {
  val = !val;
}

void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(10);
  while (!Serial)
    delay(10);

  Wire.begin();
  Wire.setClock(400000);

  Serial1.begin(TFmini::DEFAULT_BAUDRATE);

  while (!ss.init()){
    delay(1000);
    Serial.println("sensor suite error");
  }

  // cntl.init();
  bool ret = motors.init(BOARD_MOTOR_PINS);
  motors.arm();

  msgparser.serial = &Serial;
  msgparser.yivo = &yivo;

  streamTelemetry.set(true);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, RISING);
  pinMode(tonePin, OUTPUT);
  tone(tonePin, Frequency);
}

void check_serial(Stream *serial) {

  while (serial->available()) {
    int inByte = serial->read();
    if (inByte == '\r' || inByte == '\n') continue; // do nothing
    else if (inByte == '$') {
      msgparser.parse(inByte);
      uint8_t id = yivo.get_buffer_msgid();
      // do something with packet
      if (id == 50) {
        twist_t msg = yivo.unpack<twist_t>();
        cmd_direct_quad_t cmd = cntl.command(msg);
        motors.set(cmd);
      }
    }
    // else if (inByte == 'a') motors.arm();
    else if (inByte == 'p') {
      ping_t msg;
      yivo.pack(MSG_PING, reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
      serial->write(yivo.get_buffer(), yivo.get_total_size());
    }
    else if (inByte == 's') motors.stop();
    else if (inByte == 't') streamTelemetry.set(true);
    else if (inByte == 'T') streamTelemetry.set(false);
  }
}

void loop() {
  bool update = ss.check();
  if (update) {
    yivo.pack(MSG_IMU_FULL, reinterpret_cast<uint8_t *>(&ss.imu), sizeof(imu_full_t));
    Serial.write(yivo.get_buffer(), yivo.get_total_size());
  }
  check_serial(&Serial);
}
