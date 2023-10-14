#include <iostream>
#include <ostream>
#include <serialcomm/serialcomm.hpp>
#include <yivo.hpp>
#include <gecko2.hpp>
#include <squaternion.hpp> // QCF
#include <fstream>
#include <messages.hpp>
#include "MadgwickAHRS.hpp"
#include "quadcopter.hpp"
#include "command.hpp"

using namespace std;

#if defined(linux)
  string port{"/dev/cu.usbmodem14601"};
#elif defined(__APPLE__)
  // string port{"/dev/tty.usbmodem14401"};
  string port;
  bool found_port = exec("ls /dev/tty.usbmodem*", port);
#endif


// template<>
std::ostream& operator<<(std::ostream &os, vect_t<double> const &v) {
  os << "x: " << v.x << " y: " << v.y << " z: " << v.z;
  return os;
}

// Updated<quad::imu_t> gimu;

Yivo yivo;

quad::imu_t imu;
satnav_t gps;
AHRS<double> ahrs(0.1); // 1.0
QCF<double> qcf(0.05);
TiltCompass<double> tc;

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
  if (found_port == false) {
    cout << "<<< Couldn't find serial port >>>" << endl;
    exit(1);
  }
  cout << "* Opening serial port: " << port << endl;
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
  static uint64_t count = 0;

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

#if 0
          vec_t a;
          vect_t g;
          vec_t m;
#else
          vect_t<double> a;
          vect_t<double> g;
          vect_t<double> m;
#endif

#if 0
          a.x = ac[0] * imu.accel.x + ac[1] * imu.accel.y + ac[2] * imu.accel.z + ac[3];
          a.y = ac[4] * imu.accel.x + ac[5] * imu.accel.y + ac[6] * imu.accel.z + ac[7];
          a.z = ac[8] * imu.accel.x + ac[9] * imu.accel.y + ac[10] * imu.accel.z + ac[11];

          g.x = (imu.gyro.x - gc[0]);
          g.y = (imu.gyro.y - gc[1]);
          g.z = (imu.gyro.z - gc[2]);

          m.x = mc[0] * imu.mag.x - mc[3]; // uT
          m.y = mc[1] * imu.mag.y - mc[4];
          m.z = mc[2] * imu.mag.z - mc[5];
#else
          a.x = imu.accel.x;
          a.y = imu.accel.y;
          a.z = imu.accel.z;

          g.x = imu.gyro.x;
          g.y = imu.gyro.y;
          g.z = imu.gyro.z;

          m.x = imu.mag.x; // uT
          m.y = imu.mag.y;
          m.z = imu.mag.z;
#endif
          // ahrs.update(a, g, m, 0.01);
          // ahrs.update(a, g, 0.01);
          // publish(ahrs.q);

          // qcf.update(a, g, 0.01);
          // publish(qcf.q);

          if (count++ % 50 == 0) {
            // cout << "a: " << a << "  m: " << m << endl;
            cout << "a: " << a << "  g: " << g << endl;
            // cout << "a: " << a << "  g: " << g << "  m: " << m  << endl;
          }

          // publish(tc.update(a, m));
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