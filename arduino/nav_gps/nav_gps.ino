
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <squaternion.hpp>
#include <gciSensors.hpp>
#include <messages.hpp>

// constexpr float knots2mps = 0.514444;
constexpr bool RAW = true;
#define DEBUG 0

// struct __attribute__((packed)) date_t {
//   uint8_t year, month, day;
// }; // 3*1 = 3

// // clock_time_t t;
// struct __attribute__((packed)) clock_time_t {
//   uint8_t hour, minute, seconds;
// }; // 3*1 = 3

// struct __attribute__((packed)) vec_t {
//   float x,y,z;
// }; // 3*4 = 12

// struct __attribute__((packed)) quat_t {
//   float w,x,y,z;
// }; // 4*4 = 16

// struct __attribute__((packed)) gps_t {
//   float lat, lon; // decimal degrees
//   float altitude; // meters above MSL
//   // float speed; // meters/sec
//   float hdop; // horizontal dilution of precision
//   uint8_t satellites;
//   uint8_t fix;
//   date_t date;
//   clock_time_t time;
// }; // 4*4+2+3+3 = 24

// struct __attribute__((packed)) msg_t {
//   vec_t a; // 12 [0:11]
//   vec_t g; // 12 [12:23]
//   vec_t m; // 12 [24:35]
//   quat_t q; // 16 [36:51]
//   gps_t gps; // 24 [52:69]
//   float temperature; // 4 [76:79]
//   float pressure; // 4 [80:83]
//   uint32_t timestamp; // 4 [84:87]
// }; // 36+16+18+12+3+3 = 88

// msg_t data = {0};

uint32_t epoch = millis();
uint32_t ts = epoch;

QCF qcf(0.02);

float sm[3][4]{
  { 1.00268927, -0.00056029, -0.00190925, -0.00492348},
  {-0.00138898,  0.99580818, -0.00227335,  0.00503835},
  {-0.01438271,  0.00673172,  0.9998954 , -0.01364759}};
float gbias[3]{-0.00889949, -0.00235061, -0.00475294};
float mbias[3]{-13.15340002, 29.7714855, 0.0645215};
float mm[3]{0.96545537,0.94936676,0.967698};

LSM6DSOX::gciLSM6DSOX sox(&Wire);   // accel and gyro
LIS3MDL::gciLIS3MDL lis3mdl(&Wire); // magnetometer
BMP390::gciBMP390 bmp(&Wire);       // pressure

Adafruit_GPS GPS(&Serial1);

void setup() {
  //while (!Serial);
  Serial.begin(1000000);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_BAUD_115200);

  delay(500);
  GPS.begin(115200);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // $PGTOP,11,value*checksum
  // Value:
  // 1=>Active Antenna Shorted
  // 2=>Using Internal Antenna
  // 3=>Using Active Antenna
  GPS.sendCommand(PGCMD_ANTENNA);
  // GPS.sendCommand(PGCMD_NOANTENNA);

  delay(1000);

  // Ask for firmware version
  // Serial1.println(PMTK_Q_RELEASE);


  // i2c, fast mode
  Wire.begin();
  Wire.setClock(400000);

  sox.init();
  lis3mdl.init();
  bmp.init();
}

void loop() {
  msg_t data = {0};

  // // read data from the GPS in the 'main loop'
  // char c = GPS.read();
  // // if you want to debug, this is a good time to do it!
  // // if (0)
  // //   if (c) Serial.print(c);
  // // if a sentence is received, we can check the checksum, parse it...
  // if (GPS.newNMEAreceived()) {
  //   // a tricky thing here is if we print the NMEA sentence, or data
  //   // we end up not listening and catching other sentences!
  //   // so be very wary if using OUTPUT_ALLDATA and trying to print out data
  //   // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
  //   if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
  //     return; // we can fail to parse a sentence in which case we should just wait for another
  // }
  // else return;

  // Serial.printf(">> fix: %d sats: %d ant: %d\n",
  //   GPS.fix, GPS.satellites, GPS.antenna);
  // // Serial.printf(">> pos: %f, %f alt: %f\n",
  // //   GPS.latitude, GPS.longitude, GPS.altitude);
  // Serial.print(GPS.latitudeDegrees);
  // Serial.print(", ");
  // Serial.print(GPS.longitudeDegrees);
  // Serial.print(" alt: ");
  // Serial.println(GPS.altitude);
  // Serial.printf(">> Date: %d-%d-%d Time: %d:%d:%d\n",
  //   GPS.year, GPS.month, GPS.day,
  //   GPS.hour, GPS.minute, GPS.seconds);
  // return;

  GPS.read();
  uint32_t now = millis();
  while ((now - ts) < 10) {
    // delay(1);
    GPS.read();
    now = millis();
  }
  float dt = (now - ts) * 0.001;
  ts = now;

  data.timestamp = now - epoch;

  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
    data.gps.fix = GPS.fix;
    if (GPS.fix) {
      data.gps.lat = GPS.latitudeDegrees; // decimal degrees
      data.gps.lon = GPS.longitudeDegrees; // decimal degrees
      data.gps.altitude = GPS.altitude; // meters above MSL
      data.gps.satellites = GPS.satellites;
      // data.gps.speed = GPS.speed * knots2mps; // RMC meters/sec, only do GGA
      data.gps.hdop = GPS.HDOP;

      data.gps.date.month = GPS.month;
      data.gps.date.day = GPS.day;
      data.gps.date.year = GPS.year;
      data.gps.time.hour = GPS.hour;
      data.gps.time.minute = GPS.minute;
      data.gps.time.seconds = GPS.seconds;

#if DEBUG == 1

      Serial.printf(">> fix: %d sats: %d ant: %d\n",
        GPS.fix, GPS.satellites, GPS.antenna);

      Serial.print(">> pos: ");
      Serial.print(GPS.latitudeDegrees);
      Serial.print("deg, ");
      Serial.print(GPS.longitudeDegrees);
      Serial.print("deg alt: ");
      Serial.print(GPS.altitude);
      Serial.print("m HDOP: ");
      Serial.print(GPS.HDOP);
      Serial.println("m");

      Serial.printf(">> Date: %d-%d-%d Time: %d:%d:%d\n",
        GPS.year, GPS.month, GPS.day,
        GPS.hour, GPS.minute, GPS.seconds);

      Serial.println("------------------------------------");
#endif
    }
  }

  LSM6DSOX::sox_t s = sox.read();

  #if RAW
  data.a.x = s.ax; // g
  data.a.y = s.ay;
  data.a.z = s.az;
  #else
  data.a.x = sm[0][0] * s.ax + sm[0][1] * s.ay + sm[0][2] * s.az + sm[0][3];
  data.a.y = sm[1][0] * s.ax + sm[1][1] * s.ay + sm[1][2] * s.az + sm[1][3];
  data.a.z = sm[2][0] * s.ax + sm[2][1] * s.ay + sm[2][2] * s.az + sm[2][3];
  #endif

  #if RAW
  data.g.x = s.gx; // rad/s
  data.g.y = s.gy;
  data.g.z = s.gz;
  #else
  data.g.x = (s.gx - gbias[0]);
  data.g.y = (s.gy - gbias[1]);
  data.g.z = (s.gz - gbias[2]);
  #endif

  // data.temperature = s.temp; // C

  // uint32_t now = millis();
  // float dt = (now - ts) * 0.001;
  // ts = now;
  #if RAW
  Quaternion q;
  #else
  Quaternion q = qcf.update(
    data.a.x, data.a.y, data.a.z,
    data.g.x, data.g.y, data.g.z,
    dt);
  #endif

  data.q.w = q.w;
  data.q.x = q.x;
  data.q.y = q.y;
  data.q.z = q.z;


  LIS3MDL::mag_t mag = lis3mdl.read();
  #if RAW
  data.m.x = mag.x; // uT
  data.m.y = mag.y;
  data.m.z = mag.z;
  #else
  data.m.x = mm[0] * mag.x - mbias[0]; // uT
  data.m.y = mm[1] * mag.y - mbias[1];
  data.m.z = mm[2] * mag.z - mbias[2];
  #endif

  BMP390::pt_t pt = bmp.read();
  if (pt.ok) {
    data.pressure = pt.press; // Pa
    data.temperature = pt.temp; // C
  }

  if (DEBUG == 0) Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(data));
}
