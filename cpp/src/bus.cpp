#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <yivo.hpp>
#include <gecko2.hpp>
#include <fstream>
#include <messages.hpp>
// #include <string.h>

using namespace std;

#if defined(linux)
  string port{"/dev/cu.usbmodem14601"};
#elif defined(__APPLE__)
  string port{"/dev/cu.usbmodem14601"};
#endif

constexpr uint16_t imu_size = sizeof(imu_agmpt_t);

bool get_msg(void* buffer, uint8_t msgid, uint16_t msgsize, Yivo& yivo) {
    const uint8_t found_id = yivo.get_buffer_msgid();
    const uint16_t size = yivo.get_payload_size();

    if (msgsize != size || found_id != msgid) return false;

    memcpy(buffer, yivo.get_payload_buffer(), msgsize);
    return true;
}

int main() {
  ofstream out("bus.out.file");

  Yivo yivo;

  imu_agmpt_t imu;

  SerialPort ser;
  // ser.open(port, B1000000);

  while (true) {
    int d = ser.read();
    if (d >= 0) {
      bool ok = yivo.read(d);
      if (ok) {
        const uint8_t msgid = yivo.get_buffer_msgid();

        switch (msgid) {
          case 1:
            bool ok = get_msg(&imu, MSG_IMU_AGMPT, imu_size, yivo);
        }
      }
    }

    // cout << "bus" << endl;
    gecko::msleep(1000);
  }
}