#include <Wire.h>

#include <vector>
#include <gciSensors.hpp>
#include <Adafruit_NeoPixel.h>
#include <yivo.hpp>
#include <gecko2.hpp>
#include <gecko2/boards/adafruit/Adafruit_ItsyBitsy_M0.hpp>

Yivo<128> yivo; // uC to computer packetizer



Serializer msp;

class gciTFmini:public cSensor {
  public: 
  gciTFmini(uint8_t id) {
    msg.id = id;
  }
  bool init(){
    msg.min_distance = 20;
    msg.max_distance = 1200;
    // msg.id = 0;
    msg.type = Distance_t::LIDAR;

    tfmini.attach(Serial1);
    return true;
  }
  void read() {
    if (tfmini.available()) {
      msg.distance = tfmini.getDistance();
      msg.timestamp = millis();
    }
  }

  void send_msg() {
    if(!found) return;
    yivo.pack_n_send(RANGE, reinterpret_cast<uint8_t*>(&msg), sizeof(Distance_t));

  }

  protected:
  TFmini tfmini;
  Distance_t msg;
};

constexpr int IMU_USE_UNCALIBRATED_DATA = 0;

class gciLSOXLISBMP: public cSensor {
  public:

    gciLSOXLISBMP():
        soxFound(false), magFound(false), pressFound(false),
        qcf(0.02),
        sm{{ 1.00268927, -0.00056029, -0.00190925, -0.00492348},
            {-0.00138898,  0.99580818, -0.00227335,  0.00503835},
            {-0.01438271,  0.00673172,  0.9998954 , -0.01364759}},
        gbias{-0.00889949 -0.00235061 -0.00475294},
        mbias{-13.15340002, 29.7714855, 0.0645215},
        mm{0.96545537,0.94936676,0.967698},
        sox(&Wire), lis3mdl(&Wire), bmp(&Wire) {}

    bool init() {
        if (sox.init()) {
            soxFound = true;

            // Accelerometer ------------------------------------------
            // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            // sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

            // Gyros ----------------------------------------------------
            // sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            // sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
        }

        // Magnetometer -----------------------------------------------------
        if (lis3mdl.init()) {
            magFound = true;
            // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
            // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
            // already does this
            // lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
            // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
            // lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        }

        if (bmp.init()) {
          pressFound = true;
        }

        found = magFound && soxFound && pressFound;
        ts = millis();
        return found;
    }

    void read() {

        if (soxFound) {
            LSM6DSOX::sox_t s = sox.read();

            #if IMU_USE_UNCALIBRATED_DATA
            msg.ax = s.ax; // g
            msg.ay = s.ay;
            msg.az = s.az;
            #else
            msg.ax = sm[0][0] * s.ax + sm[0][1] * s.ay + sm[0][2] * s.az + sm[0][3];
            msg.ay = sm[1][0] * s.ax + sm[1][1] * s.ay + sm[1][2] * s.az + sm[1][3];
            msg.az = sm[2][0] * s.ax + sm[2][1] * s.ay + sm[2][2] * s.az + sm[2][3];
            #endif

            #if IMU_USE_UNCALIBRATED_DATA
            msg.gx = s.gx; // rad/s
            msg.gy = s.gy;
            msg.gz = s.gz;
            #else
            msg.gx = (s.gx - gbias[0]);
            msg.gy = (s.gy - gbias[1]);
            msg.gz = (s.gz - gbias[2]);
            #endif

            msg.temperature = s.temp; // C

            uint32_t now = millis();
            dt = (now - ts) * 0.001;
            ts = now;

            q = qcf.update(
              msg.ax, msg.ay, msg.az, 
              msg.gx, msg.gy, msg.gz, 
              dt);

            msg.qw = q.w;
            msg.qx = q.x;
            msg.qy = q.y;
            msg.qz = q.z;

            msg.timestamp = now; // time msec
        }

        if (magFound) {
            LIS3MDL::mag_t s = lis3mdl.read();

            #if IMU_USE_UNCALIBRATED_DATA
            msg.mx = s.x; // uT
            msg.my = s.y;
            msg.mz = s.z;
            #else
            msg.mx = mm[0] * s.x - mbias[0]; // uT
            msg.my = mm[1] * s.y - mbias[1];
            msg.mz = mm[2] * s.z - mbias[2];
            #endif
        }

        if (pressFound) {
          BMP390::pt_t s = bmp.read();
          if (s.ok) {
            msg.pressure = s.press;
            msg.temperature = s.temp;
          }
          else {
            msg.pressure = 0.0f;
            msg.temperature = 0.0f;
          }
        }
    }

  void send_msg() {
    if(!found) return;
    yivo.pack_n_send(IMU_AGMQPT, reinterpret_cast<uint8_t*>(&msg), sizeof(ImuAGMQPT_t));
    // func(100, reinterpret_cast<uint8_t*>(&msg), sizeof(ImuA_t));
  }

  protected:
    bool soxFound;
    bool magFound;
    bool pressFound;

    ImuAGMQPT_t msg;

    QCF qcf;

    LSM6DSOX::gciLSM6DSOX sox;   // accel and gyro
    LIS3MDL::gciLIS3MDL lis3mdl; // magnetometer
    BMP390::gciBMP390 bmp;       // pressure
    Quaternion q;
    float dt;       // time difference between samples
    uint32_t ts;    // timestamp (msec)
    float sm[3][4]; // accel scale/bias
    float gbias[3]; // gyro bias
    float mbias[3]; // mag bias
    float mm[3];    // mag scale
};

///////////////////



/*
Neopixel
- blue: not armed
- green: armed
- red: issue
*/
class System {
  public:
  System() {}

  bool init(uint16_t hb_sensors) {
    bool ret = true;

    for (auto& s : sensors) {
      ret &= s->init();
    }
    ret &= blackbox.init();
    ret &= motors.init(BOARD_MOTOR_PINS);
    ret &= heartbeat.init(hb_sensors);
    ret &= autopilot.init();

    #if HAS_NEOPIXEL
    // turn off
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    #endif

    ret = true;

    return ret;
  }

  void check_sensors() {

    for (auto& s : sensors) {
      s->read();
      if (streamTelemetry) s->send_msg();
    }
  }

  void loop() {
    check_sensors();

    if (Serial.available()) {
      int inByte = Serial.read();
      if (inByte == '\r' || inByte == '\n'); // do nothing
      else if (inByte == 'a') motors.arm();
      else if (inByte == 'g') yivo.pack_n_send(PING, nullptr, 0);
      else if (inByte == 'm') {
        Motors4_t m = motors.get_msg();
        yivo.pack_n_send(MOTORS, reinterpret_cast<uint8_t*>(&m), 10);
      }
      else if (inByte == 'p') { // pwm motor command
          // p,m0,m1,m2,m3
          int m[4];
          for (int i=0; i < 4; i++) {
            uint8_t a = Serial.read(); // low byte
            uint8_t b = Serial.read(); // high byte
            m[i] = (b << 8) + a; 
          }
          motors.set(m[0],m[1],m[2],m[3]);
      }
      else if (inByte == 'r') motors.ramp(); // remove? Why do this at the embedded level?
      else if (inByte == 's') motors.stop();
      else if (inByte == 't') streamTelemetry.toggle(); //telemetry = !telemetry;
      else if (inByte == 'T') check_sensors();
      else yivo.pack_n_send(YIVO_ERROR); // send error, unknown command
    }

    if (streamTelemetry) {

      if (heartbeat.check()){
        Heartbeat_t msg = heartbeat.get_msg();
        yivo.pack_n_send(HEARTBEAT, reinterpret_cast<uint8_t*>(&msg), sizeof(Heartbeat_t));
      }

      if (motors.check()) {
        Motors4_t m = motors.get_msg();
        yivo.pack_n_send(MOTORS, reinterpret_cast<uint8_t*>(&m), 10);
      }
    }


    #if HAS_NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(0, 0, 150));
    pixels.show();
    #endif
  }

  void send_msg() {} // another template?

  std::vector<cSensor*> sensors;

  protected:
  Toggle streamTelemetry;
  QuadESC motors;
  Heartbeat heartbeat;
  Autopilot autopilot;
  Blackbox blackbox;

  #if HAS_NEOPIXEL
  constexpr int LED_COUNT = 1; // How many NeoPixels are attached to the Arduino?
  Adafruit_NeoPixel pixels(LED_COUNT, BOARD_NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);
  #endif
};

// quadcopter system
System q;

// sensors
gciLSOXLISBMP imu;
gciTFmini tfmini(1);

// System q;
// gciLSOXLISBMP imu;
// gciTFmini tfmini(1);

////////////////////////////

void setup() {
    Serial.begin(1000000);
    Serial.setTimeout(5);
    while (!Serial)
        delay(10);
    
    Serial1.begin(TFmini::DEFAULT_BAUDRATE);
    
    q.sensors.push_back(&imu);
    q.sensors.push_back(&tfmini);

    // for heartbeat
    uint16_t sensors = Heartbeat_t::ACCEL;
    sensors |= Heartbeat_t::GYRO;
    sensors |= Heartbeat_t::BAROMETER;
    sensors |= Heartbeat_t::TEMPERATURE;
    sensors |= Heartbeat_t::RANGE;
    sensors |= Heartbeat_t::BATTERY;
    sensors |= Heartbeat_t::COMPASS;
    bool ready = q.init(sensors);

    while (!ready) {
      Serial.println("Error initializing System"); // change to yivo message?
      delay(1000);
    }
}

void loop() {
  // Serial.println("hello");

  q.loop();
}





// class cSensor {
//   public:
//   cSensor(): found(false) {}
//   virtual bool init() =0;
//   virtual void read() =0;
//   virtual void send_msg() =0;

//   bool found;
//   // uint8_t msg_id;
//   // std::function<void(uint8_t,uint8_t*,std::size_t)> func;
//   // std::function<void(Yivo<128>&,uint8_t,uint8_t*,std::size_t)> func;
// };


// class tImu: public cSensor {
//   public:
//   tImu(): cSensor() {}
//   bool init() {
//     found = true;
//     return true;
//   }
//   void read() {}
//   void send_msg() {
//     if(!found) return;
//     yivo.pack_n_send(100, reinterpret_cast<uint8_t*>(&msg), sizeof(ImuA_t));
//     // func(100, reinterpret_cast<uint8_t*>(&msg), sizeof(ImuA_t));
//   }

//   ImuA_t msg;
//   // std::function<void(uint8_t,uint8_t*,std::size_t)> func;
// };

