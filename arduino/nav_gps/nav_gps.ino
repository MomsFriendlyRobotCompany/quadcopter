
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <squaternion.hpp>
#include <gciSensors.hpp>
#include <messages.hpp>
#include <yivo.hpp>

// constexpr float knots2mps = 0.514444;
constexpr bool RAW = true;
#define DEBUG 0

uint32_t epoch = millis();
uint32_t ts = epoch;

LSM6DSOX::gciLSM6DSOX sox(&Wire);   // accel and gyro
LIS3MDL::gciLIS3MDL lis3mdl(&Wire); // magnetometer
BMP390::gciBMP390 bmp(&Wire);       // pressure

Adafruit_GPS GPS(&Serial1);
Yivo yivo;

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

  // i2c, fast mode
  Wire.begin();
  Wire.setClock(400000);

  sox.init();
  lis3mdl.init();
  bmp.init();
}

void loop() {

  GPS.read();
  uint32_t now = millis();
  while ((now - ts) < 10) {
    // delay(1);
    GPS.read();
    now = millis();
  }
  // // float dt = (now - ts) * 0.001;
  ts = millis();

  if (GPS.newNMEAreceived()) {
    gps_t gps;
    GPS.parse(GPS.lastNMEA());
    if (GPS.fix) {
      gps.lat = GPS.latitudeDegrees; // decimal degrees
      gps.lon = GPS.longitudeDegrees; // decimal degrees
      gps.altitude = GPS.altitude; // meters above MSL
      gps.satellites = GPS.satellites;
      gps.fix = GPS.fix;
      gps.hdop = GPS.HDOP;

      gps.date.month = GPS.month;
      gps.date.day = GPS.day;
      gps.date.year = GPS.year;
      gps.time.hour = GPS.hour;
      gps.time.minute = GPS.minute;
      gps.time.seconds = GPS.seconds;

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
#else
      YivoPack_t p = yivo.pack(MSG_SATNAV, reinterpret_cast<uint8_t *>(&gps), sizeof(gps_t));
      Serial.write(p.buffer, p.size);
#endif
    }
  }

  imu_agmpt_t imu;
  imu.timestamp = millis() - epoch;

  LSM6DSOX::sox_t s = sox.read();
  
  imu.a.x = s.ax; // g
  imu.a.y = s.ay;
  imu.a.z = s.az;
  
  imu.g.x = s.gx; // rad/s
  imu.g.y = s.gy;
  imu.g.z = s.gz;
  
  LIS3MDL::mag_t mag = lis3mdl.read();
  
  imu.m.x = mag.x; // uT
  imu.m.y = mag.y;
  imu.m.z = mag.z;

  BMP390::pt_t pt = bmp.read();
  if (pt.ok) {
    imu.pressure = pt.press; // Pa
    imu.temperature = pt.temp; // C
  }

  if (DEBUG == 0) {
    YivoPack_t p = yivo.pack(MSG_IMU_AGMPT, reinterpret_cast<uint8_t *>(&imu), sizeof(imu_agmpt_t));
    // uint8_t *buff = yivo.get_buffer();
    // uint16_t size = yivo.get_total_size();
    Serial.write(p.buffer, p.size);
  }
}
