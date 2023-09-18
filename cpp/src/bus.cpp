#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <yivo.hpp>
#include <gecko2.hpp>
#include <fstream>
#include <messages.hpp>
#include "MadgwickAHRS.hpp"
#include "quad_msgs.hpp"

using namespace std;

#if defined(linux)
  string port{"/dev/cu.usbmodem14601"};
#elif defined(__APPLE__)
  string port{"/dev/tty.usbmodem14501"};
#endif

// template<typename T>
// class Updated {
//   public:
//   Updated(): updated(false) {}

//   const T get() {
//     updated = false;
//     return value;
//   }

//   void set(const T& v) {
//     value = v;
//     updated = true;
//   }

//   explicit operator bool() const {
//     return updated;
//   }

//   protected:
//   T value;
//   bool updated;
// };


Updated<quad::imu_t> gimu;
Updated<quad::joystick_t> js;

Yivo yivo;

// quad::imu_t imu;
imu_agmpt_t imu;
satnav_t gps;
AHRS ahrs(1.0);

SerialPort ser;

// double ac[3][4]{{ 1.00268927, -0.00056029, -0.00190925, -0.00492348},
//     {-0.00138898,  0.99580818, -0.00227335,  0.00503835},
//     {-0.01438271,  0.00673172,  0.9998954 , -0.01364759}};
// double gbias[3]{-0.00889949 -0.00235061 -0.00475294};
// double mbias[3]{-13.15340002, 29.7714855, 0.0645215};
// double mm[3]{0.96545537,0.94936676,0.967698};

double ac[12]{
   1.00268927, -0.00056029, -0.00190925, -0.00492348,
  -0.00138898,  0.99580818, -0.00227335,  0.00503835,
  -0.01438271,  0.00673172,  0.9998954 , -0.01364759};
double gc[3]{-0.00889949 -0.00235061 -0.00475294};
double mc[6]{
    0.96545537,  0.94936676,0.967698,
  -13.15340002, 29.7714855, 0.0645215};

void publish(const Quaternion& q) {
  double r, p, y;
  q.to_euler(&r,&p,&y,true);
  cout << r << " " << p << " " << y << " -> " << q << endl;
}

void publish(const satnav_t& gps) {
  cout << gps.lat << " " << gps.lon << endl;
}

void read_sensors() {

  int avail = ser.available();
  // cout << avail << flush;
  if (avail <= 6) return;

  uint8_t msgid = 0;
  for (int i = 0; i < avail; ++i) {
    char c = ser.read();
    msgid = yivo.read(c);
    if (msgid > 0) break;
  }
  // cout << "." << flush;
  if (msgid == 0) return;

  switch (msgid) {
    case quad::IMU:
      imu = yivo.unpack<imu_agmpt_t>();

      vec_t a;
      a.x = ac[0] * imu.a.x + ac[1] * imu.a.y + ac[2] * imu.a.z + ac[3];
      a.y = ac[4] * imu.a.x + ac[5] * imu.a.y + ac[6] * imu.a.z + ac[7];
      a.z = ac[8] * imu.a.x + ac[9] * imu.a.y + ac[10] * imu.a.z + ac[11];

      vec_t g;
      g.x = (imu.g.x - gc[0]);
      g.y = (imu.g.y - gc[1]);
      g.z = (imu.g.z - gc[2]);

      vec_t m;
      m.x = mc[0] * imu.m.x - mc[3]; // uT
      m.y = mc[1] * imu.m.y - mc[4];
      m.z = mc[2] * imu.m.z - mc[5];

      ahrs.update(a, g, m, 0.01);
      publish(ahrs.q);
      break;

    case MSG_SATNAV:
      gps = yivo.unpack<satnav_t>();
      publish(gps);
      break;
  }
}

void get_js() {
  ;
}

void control() {
  ;
}

void loop() {
  ser.open(port, B1000000);

  // read config file

  // setup calibration ----------------
  while (true) {
    quad::continue_t c;
    YivoPack_t m = yivo.pack(quad::CONTINUE, reinterpret_cast<uint8_t *>(&c), sizeof(c));
    ser.write(m.data(), m.size());

    uint8_t msgid = 0;
    int avail = ser.available();
    if (avail <= 6) continue;

    for (int i = 0; i < avail; ++i) {
      char c = ser.read();
      msgid = yivo.read(c);
      if (msgid > 0) break;
    }

    if (msgid == quad::HEART_BEAT) {
      quad::heartbeat_t h = yivo.unpack<quad::heartbeat_t>();
      break; // break out of setup loop
    }

    gecko::msleep(10);
  }

  // main loop --------------------
  while (true) {
    read_sensors();
    get_js();
    control();
    gecko::msleep(1);
  }
}

int main() {
  loop();
}