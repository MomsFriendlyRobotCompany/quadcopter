#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <yivo.hpp>
#include <gecko2.hpp>
#include <fstream>
#include <messages.hpp>
#include "MadgwickAHRS.hpp"

using namespace std;

#if defined(linux)
  string port{"/dev/cu.usbmodem14601"};
#elif defined(__APPLE__)
  string port{"/dev/tty.usbmodem14501"};
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
  // ofstream out("bus.out.file");
  double sm[3][4]{{ 1.00268927, -0.00056029, -0.00190925, -0.00492348},
      {-0.00138898,  0.99580818, -0.00227335,  0.00503835},
      {-0.01438271,  0.00673172,  0.9998954 , -0.01364759}};
  double gbias[3]{-0.00889949 -0.00235061 -0.00475294};
  double mbias[3]{-13.15340002, 29.7714855, 0.0645215};
  double mm[3]{0.96545537,0.94936676,0.967698};

  Yivo yivo;

  imu_agmpt_t imu;
  AHRS ahrs(1.0); // backwards?  0 0 0 1???

  SerialPort ser;
  ser.open(port, B1000000);

  uint8_t buffer[128];

  while (true) {
    int avail = ser.available();
    // cout << avail << flush;
    if (avail > 6) {
      bool ok = false;
      for (int i = 0; i < avail; ++i) {
        char c = ser.read();
        ok = yivo.read(c);
      }
      // cout << "." << flush;
      if (ok) {
        // cout << "x" << flush;
        const uint8_t msgid = yivo.get_buffer_msgid();

        switch (msgid) {
          case MSG_IMU_AGMPT:
            imu = yivo.unpack<imu_agmpt_t>();

            // vec_t a;
            // a.x = sm[0][0] * imu.a.x + sm[0][1] * imu.a.y + sm[0][2] * imu.a.z + sm[0][3];
            // a.y = sm[1][0] * imu.a.x + sm[1][1] * imu.a.y + sm[1][2] * imu.a.z + sm[1][3];
            // a.z = sm[2][0] * imu.a.x + sm[2][1] * imu.a.y + sm[2][2] * imu.a.z + sm[2][3];

            // vec_t g;
            // g.x = (imu.g.x - gbias[0]);
            // g.y = (imu.g.y - gbias[1]);
            // g.z = (imu.g.z - gbias[2]);

            // vec_t m;
            // m.x = mm[0] * imu.m.x - mbias[0]; // uT
            // m.y = mm[1] * imu.m.y - mbias[1];
            // m.z = mm[2] * imu.m.z - mbias[2];

            vec_t a{0,0,1}, g{0,0,0.00}, m{ -39.038291931152344, 18.211050033569336, -53.50774383544922};

            ahrs.update(a, g, m, 0.01);
            // cout << ahrs.q << endl;
            double r, p, y;
            ahrs.q.to_euler(&r,&p,&y,true);
            cout << r << " " << p << " " << y << " -> " << ahrs.q << endl;
        }
      }
    }

    // cout << "bus" << endl;
    gecko::msleep(1);
  }
}