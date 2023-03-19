#pragma once

#include <gecko2.hpp>

class Controller {
  public:

  bool init(const uint8_t motor_pins[4]) {
    bool ret = false;
    ret = motors.init(motor_pins);
    motors.arm();

    return ret;
  }

  void command(cmd_pose_quad_t& cmd) {
    motors.set(
      MOTOR_ZERO_LEVEL,
      MOTOR_ZERO_LEVEL,
      MOTOR_ZERO_LEVEL,
      MOTOR_ZERO_LEVEL
    );
  }

  QuadESC motors;
};