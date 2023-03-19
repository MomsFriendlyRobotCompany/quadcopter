// #pragma once

// #include <gciSensors.hpp>
// #include <yivo.hpp>
// #include <gecko2/sensor.hpp>
// #include <gecko2/messages.hpp>


// class gciTFmini : public cSensor {
// public:
//   gciTFmini(uint8_t id) { msg.id = id; }

//   bool init() {
//     msg.min_distance = 20;
//     msg.max_distance = 1200;
//     msg.type         = distance_t::LIDAR;

//     tfmini.attach(*BOARD_ALT_SERIAL); // FIIXME: make stream

//     return true;
//   }

//   void read() {
//     if (tfmini.available()) {
//       msg.distance  = tfmini.getDistance();
//       msg.timestamp = millis();
//     }
//   }

//   void get_msg(Yivo &y) {
//     y.pack(MSG_DISTANCE, reinterpret_cast<uint8_t *>(&msg), sizeof(distance_t));
//   }

//   // const distance_t send_msg() const {
//   //   // if(!found) return;
//   //   // yivo.pack_n_send(RANGE, reinterpret_cast<uint8_t*>(&msg),
//   //   // sizeof(Distance_t));
//   //   return msg;
//   // }

// protected:
//   TFmini tfmini;
//   distance_t msg;
// };