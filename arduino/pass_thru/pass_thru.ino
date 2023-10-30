
#include <Wire.h>
#include <gcigps.hpp>
#include <squaternion.hpp>
#include <gciSensors.hpp>
#include <messages.hpp>
#include <yivo.hpp>
#include <quadcopter.hpp>
#include <gecko2.hpp>

#define YIVO FALSE

using gecko::mtime;
using gecko::utime;
using gecko::msleep;
using gecko::usleep;
using gecko::sleep;


// https://arduino-pico.readthedocs.io/en/latest/serial.html
// Serial1 is UART0, and Serial2 is UART1
// #define gpscomm Serial2

LSM6DSOX::gciLSM6DSOX sox(&Wire);   // accel and gyro
LIS3MDL::gciLIS3MDL lis3mdl(&Wire); // magnetometer
BMP390::gciBMP390 bmp(&Wire);       // pressure
gci::GPS gps;

SystemStatus_t systemStatus;

Yivo yivo;

Update heartbeatUpdate(UPDATE_1HZ);
Update inertialUpdate(UPDATE_100HZ);
Update gpsUpdate(UPDATE_2HZ);
Update ptUpdate(UPDATE_2HZ);


void setup() {
  usbcomm.begin(USB_BAUD);

  // rpicomm.setTX(BOARD_RPI_TX);
  // rpicomm.setRX(BOARD_RPI_RX);
  // rpicomm.begin(RPI_BAUD);

  gpscomm.setTX(BOARD_GPS_TX);
  gpscomm.setRX(BOARD_GPS_RX);
  gpscomm.setFIFOSize(128);
  gpscomm.begin(9600);
  gpscomm.print(GCI_BAUD_115200);

  delay(500);

  gpscomm.begin(115200);
  // gpscomm.print(GCI_RMCGGA);
  gpscomm.print(GCI_GGA);
  gpscomm.print(GCI_UPDATE_1HZ);
  gpscomm.print(GCI_NOANTENNA);

  delay(500);

  // i2c, fast mode
  Wire.begin();
  Wire.setClock(I2C_400K);

  sox.init(
    LSM6DSOX::ACCEL_RANGE_4_G, 
    LSM6DSOX::GYRO_RANGE_2000_DPS, 
    LSM6DSOX::RATE_104_HZ);
  lis3mdl.init(
    LIS3MDL::RANGE_4GAUSS,
    LIS3MDL::ODR_155HZ);
  bmp.init(BMP390::OS_MODE_PRES_16X_TEMP_2X);

  // analogReadResolution(12); // default 10 bit

  // adc_init();
  // adc_set_temp_sensor_enabled(1);
  // adc_select_input (4); 
  // int v =adc_read();
}

void send_heartbeat() {
  quad::heartbeat_t h;
  h.version = 1;
  h.health = 0;
  h.sensors = quad::SensorTypes::ACCELEROMETER 
    | quad::SensorTypes::GYROSCOPE 
    | quad::SensorTypes::MAGNOMETER 
    | quad::SensorTypes::BAROMETER 
    | quad::SensorTypes::GPS;
  h.state = 0;
  h.timestamp = millis();

  h.battery = analogRead(BOARD_BATTERY_PIN);
  // float cpu_temp = analogReadTemp();

#if YIVO == TRUE
  YivoPack_t hb = yivo.pack(quad::HEARTBEAT,reinterpret_cast<uint8_t *>(&h),sizeof(h));
  Serial.write(hb.data(), hb.size());
#else
  Serial.print(h.battery);
  Serial.println("--- HEARTBEAT ---");
#endif
}

gci::GpsID get_gps_msg() {
  int loop = gpscomm.available();
  // usbcomm.print(">> loop: ");
  // usbcomm.println(loop);
  bool ok = false;
  int c;
  while (--loop >= 0) {
    // usbcomm.print(".");
    c = gpscomm.read();
    if (c == -1) continue;
    // if (char(c) == '\r' || char(c) == '\n') usbcomm.println("RETURNS");
    // usbcomm.print(char(c));
    ok = gps.read(char(c));
    if (ok) {
      // usbcomm.println("ok == true");
      break;
    }
  }

  // if (ok) usbcomm.println("FOUND");
  // else usbcomm.println("MISSED");

  if (ok) return gps.get_id();
  return gci::GpsID::NONE;
}

void get_gps() {
  bool ok = false;
  satnav_t msg{0};
  
  gci::GpsID id = get_gps_msg();

  if (id == gci::GpsID::GGA) {
    gga_t gga{0};
    ok = gps.get_msg(gga);
    if (!ok) return;
    msg.lat = gga.lat;
    msg.lon = gga.lon;
    msg.altitude = gga.msl;
    msg.satellites = gga.num_sats;
    msg.time.hour = gga.utc.hour;
    msg.time.minute = gga.utc.minute;
    msg.time.seconds = gga.utc.seconds;
  }
  else if (id == gci::GpsID::RMC) {
    rmc_t rmc{0};
    ok = gps.get_msg(rmc);
    if (!ok) return;
    msg.lat = rmc.lat;
    msg.lon = rmc.lon;
    msg.time.hour = rmc.utc.hour;
    msg.time.minute = rmc.utc.minute;
    msg.time.seconds = rmc.utc.seconds;
    msg.date.month = rmc.date.month;
    msg.date.day = rmc.date.day;
    msg.date.year = rmc.date.year;
  }
  else {
    // usbcomm.println("X");
    return;
  }
      
#if YIVO == TRUE 
  YivoPack_t p = yivo.pack(MSG_SATNAV, reinterpret_cast<uint8_t *>(&msg), sizeof(satnav_t));
  Serial.write(p.data(), p.size());
#else
  Serial.print("gps: ");
  Serial.print(msg.lat);
  Serial.print(", ");
  Serial.println(msg.lon);
#endif
  return;
}

void get_inertial() {
  quad::imu_t imu;
  imu.timestamp = millis();
  imu.status = static_cast<uint8_t>(IMU_Status::GOOD);

  LSM6DSOX::sox_t s = sox.read();
  if (s.ok) {
    imu.accel.x = s.a.x; // g
    imu.accel.y = s.a.y;
    imu.accel.z = s.a.z;

    imu.gyro.x = s.g.x; // rad/s
    imu.gyro.y = s.g.y;
    imu.gyro.z = s.g.z;
  }
  else imu.status = static_cast<uint8_t>(IMU_Status::A_FAIL) | static_cast<uint8_t>(IMU_Status::G_FAIL);

  LIS3MDL::mag_t mag = lis3mdl.read();
  if (mag.ok) {
    imu.mag.x = mag.x; // uT
    imu.mag.y = mag.y;
    imu.mag.z = mag.z;
  }
  else imu.status |= static_cast<uint8_t>(IMU_Status::M_FAIL);

#if YIVO == TRUE 
  YivoPack_t p = yivo.pack(quad::IMU, reinterpret_cast<uint8_t *>(&imu), sizeof(quad::imu_t));
  Serial.write(p.data(), p.size());
#else
  Serial.println("imu");
#endif
}

void get_pt() {
  atmospheric_t msg;

  BMP390::pt_t pt = bmp.read();
  if (pt.ok == false) return;

  msg.pressure = pt.press; // Pa
  msg.temperature = pt.temp; // C

#if YIVO == TRUE 
  YivoPack_t p = yivo.pack(MSG_ATMOSPHERIC, reinterpret_cast<uint8_t *>(&msg), sizeof(atmospheric_t));
  Serial.write(p.data(), p.size());
#else
  Serial.print("pt: ");
  Serial.print(msg.pressure);
  Serial.print(", ");
  Serial.println(msg.temperature);
#endif
}

void loop() {
  if (gpsUpdate) get_gps();
  // if (inertialUpdate) get_inertial();
  // if (ptUpdate) get_pt();
  if (heartbeatUpdate) send_heartbeat();
  // usbcomm.println("-");
}
