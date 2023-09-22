#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <yivo.hpp>
#include <gecko2.hpp>
#include <squaternion.hpp> // QCF
#include <fstream>
#include <messages.hpp>
#include "MadgwickAHRS.hpp"
#include "quadcopter.hpp"

using namespace std;

#if defined(linux)
  string port{"/dev/cu.usbmodem14601"};
#elif defined(__APPLE__)
  string port{"/dev/tty.usbmodem14301"};
#endif

Updated<quad::imu_t> gimu;
// Updated<quad::joystick_t> js;

Yivo yivo;

quad::imu_t imu;
// imu_agmpt_t imu;
satnav_t gps;
AHRS<double> ahrs(0.1); // 1.0
QCF qcf(0.05);

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

void setup() {
  ser.open(port, B1000000);

  quad::calibration_t cal;
  for (int i=0; i < 12; ++i) cal.a[i] = static_cast<float>(ac[i]);
  for (int i=0; i < 3; ++i) cal.g[i] = static_cast<float>(gc[i]);
  for (int i=0; i < 6; ++i) cal.m[i] = static_cast<float>(mc[i]);
  YivoPack_t p = yivo.pack(quad::CALIBRATION, reinterpret_cast<uint8_t *>(&cal), sizeof(quad::calibration_t));
  ser.write(p.data(), p.size());
}

void publish(const Quaternion& q) {
  static uint64_t count = 0;
  double r, p, y;
  bool degrees = true;
  q.to_euler(&r,&p,&y,degrees);

  if (count++ % 100 == 0) {
    cout << "Attitude: " << r << " " << p << " " << y << " -> " << q << endl;
  }
}

void publish(const satnav_t& gps) {
  cout << "GPS: " << gps.lat << " " << gps.lon << endl;
}

void read_sensors() {

  int avail = ser.available();
  // cout << avail << flush;
  if (avail <= 6) return;

  uint8_t msgid = 0;
  // for (int i = 0; i < avail; ++i) {
  while (avail-- > 0) {
    char c = ser.read();
    msgid = yivo.read(c);
    // if (msgid > 0) break;
    if (msgid > 0) {
      switch (msgid) {
        case quad::IMU:
          imu = yivo.unpack<quad::imu_t>();

          vec_t a;
          a.x = ac[0] * imu.accel.x + ac[1] * imu.accel.y + ac[2] * imu.accel.z + ac[3];
          a.y = ac[4] * imu.accel.x + ac[5] * imu.accel.y + ac[6] * imu.accel.z + ac[7];
          a.z = ac[8] * imu.accel.x + ac[9] * imu.accel.y + ac[10] * imu.accel.z + ac[11];

          vec_t g;
          g.x = (imu.gyro.x - gc[0]);
          g.y = (imu.gyro.y - gc[1]);
          g.z = (imu.gyro.z - gc[2]);

          vec_t m;
          m.x = mc[0] * imu.mag.x - mc[3]; // uT
          m.y = mc[1] * imu.mag.y - mc[4];
          m.z = mc[2] * imu.mag.z - mc[5];

          // ahrs.update(a, g, m, 0.01);
          ahrs.update(a, g, 0.01);
          qcf.update(a.x,a.y,a.z, g.x,g.y,g.z, 0.01);
          // publish(ahrs.q);
          publish(qcf.q);
          break;

        case MSG_SATNAV:
          gps = yivo.unpack<satnav_t>();
          // publish(gps);
          break;

        case quad::HEARTBEAT:
          quad::heartbeat_t hb = yivo.unpack<quad::heartbeat_t>();
          // printf("--- HEARTBEAT [%u] ---\n", hb.timestamp);
      }
    }
  }
  // cout << "." << flush;
  // if (msgid == 0) return;
  // avail-=1;
}

void get_cmds() {
  ;
}

void control() {
  ;
}

void loop() {
  // ser.open(port, B1000000);

  // read config file

  // setup calibration ----------------
  // while (true) {
  //   // quad::continue_t c;
  //   // YivoPack_t m = yivo.pack(quad::CONTINUE, reinterpret_cast<uint8_t *>(&c), sizeof(c));
  //   // ser.write(m.data(), m.size());

  //   uint8_t msgid = 0;
  //   int avail = ser.available();
  //   if (avail <= 6) continue;

  //   for (int i = 0; i < avail; ++i) {
  //     char c = ser.read();
  //     msgid = yivo.read(c);
  //     if (msgid > 0) break;
  //   }

  //   if (msgid == quad::HEART_BEAT) {
  //     quad::heartbeat_t h = yivo.unpack<quad::heartbeat_t>();
  //     break; // break out of setup loop
  //   }

  //   gecko::msleep(10);
  // }

  // main loop --------------------
  while (true) {
    read_sensors();
    get_cmds();
    control();
    gecko::msleep(1);
  }
}

int main() {
  cout << "*** BUS START ***" << endl;
  setup();
  loop();
}