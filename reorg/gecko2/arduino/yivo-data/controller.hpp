#pragma once

#include <gecko2.hpp>


class MsgParser {
  public:

  bool parse(int inByte) {
    int cnt   = 127;
    bool done = false;
    while (cnt--) {
      done = yivo->read(inByte);
      if (done) break;
      inByte = serial->read();
    }
    return done;
  }

  Stream *serial;
  Yivo *yivo;
};

// class AsciiParser {
//   public:
//   bool parse(int inByte) {

//   }

//   Stream *serial;
// };

class Controller {
  public:

  bool init() {
    bool ret = true;
    return ret;
  }

  cmd_direct_quad_t command(twist_t& cmd) {
    cmd_direct_quad_t m;
    // motors.set(
    //   MOTOR_ZERO_LEVEL,
    //   MOTOR_ZERO_LEVEL,
    //   MOTOR_ZERO_LEVEL,
    //   MOTOR_ZERO_LEVEL
    // );
    return m;
  }

  cmd_direct_quad_t command(cmd_pose_quad_t& cmd) {
    cmd_direct_quad_t m;
    // motors.set(
    //   MOTOR_ZERO_LEVEL,
    //   MOTOR_ZERO_LEVEL,
    //   MOTOR_ZERO_LEVEL,
    //   MOTOR_ZERO_LEVEL
    // );
    return m;
  }

};