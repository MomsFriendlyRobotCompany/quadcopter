
#include "debug.hpp"
#include "imu.hpp"
#include "motors.hpp"
#include "packer.hpp"
#include <Wire.h>
#include <cstdint>

gciLSOXLIS imu;
gciDPS310 press;
QuadESC motors;


void motor_ramp() {
    // ramp up to max
    for (int i=MOTOR_ZERO_LEVEL; i < MOTOR_MAX_LEVEL; i+=100){
      int val = i;
      motors.set(val,val,val,val);
      delay(500);
    }

    // ramp down to min
    for (int i=MOTOR_MAX_LEVEL; i > MOTOR_ZERO_LEVEL; i-=100){
      int val = i;
      motors.set(val,val,val,val);
      delay(500);
    }

    // set to 0
    int val = MOTOR_ZERO_LEVEL;
    motors.set(val,val,val,val);
}

/* Toggle board's LED on/off */
void led(bool val) {
    constexpr int LED_PIN = 13;
    constexpr int wait_time = 500;

    if (val) digitalWrite( LED_PIN, HIGH);
    else {
        // delay(wait_time);
        digitalWrite(LED_PIN, LOW);
        // delay(wait_time); // don't go too fast
    }
}

void setup() {

    Serial.begin(115200);
    // while (!Serial)
    //     delay(10);

    // i2c, fast mode
    Wire.begin();
    Wire.setClock(400000);

    // setup sensors
    imu.init();
    press.init(); 

    Serial.println("Boot complete:");
    Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");
    Serial.println(" " + press.found ? "+ Pressure sensor ready"
                                     : "! Pressure Sensor not found");

    motors.init();
    motors.arm();
    Serial.println("Motors ready and armed");
    
}

uint32_t blink_time = 0;
bool led_blink = true;
int motor_val[4] = {1000};
int incr = 50;

void loop() {
    // if (imu.found) {
    //     imu.read();
    //     // printMag();
    //     // printGyro();
    //     // printAccel();
    //     // printQuaternion();
    //     pack_n_send(imu.id, imu.bsize, imu.data.b);
    // }

    // if (press.found) {
    //     press.read();
    //     pack_n_send(press.id, press.bsize, press.data.b);
    // }

    // serial ascii input
    if (Serial.available() > 0) {
        int inByte = Serial.read();
        
        if (inByte == 'a') { // add
            int mv = Serial.read();
            motor_val[0] = motor_limit(mv + incr);
            motors.set(motor_val[0], motor_val[1], motor_val[2], motor_val[3]);
        }
        else if (inByte == 'd') { // decrease
            int mv = Serial.read();
            motor_val[0] = motor_limit(mv - incr);
            // motor.set(motor_val, motor_val, motor_val, motor_val);
            motors.set(motor_val[0], motor_val[1], motor_val[2], motor_val[3]);
        }
        else if (inByte == 's') { // stop
            motor_val[0] = MOTOR_ZERO_LEVEL;
            motor_val[1] = MOTOR_ZERO_LEVEL;
            motor_val[2] = MOTOR_ZERO_LEVEL;
            motor_val[3] = MOTOR_ZERO_LEVEL;
            motors.set(motor_val[0], motor_val[1], motor_val[2], motor_val[3]);
        }
        // else if (inByte == 'p') { // pwm
        //     // p,motor,steering
        //     inByte = Serial.read();
        //     pos = pwmLimit(inByte);
        //     inByte = Serial.read();
        //     steer = pwmLimit(inByte);
        // }
        // else if (inByte == 'r') { // reboot
        //     pos = 90;
        //     steer = 90;
        //     reboot();
        //     delay(1000);
        // }
    }

    uint32_t now = millis();
    if (now > blink_time){
      led(led_blink);
      led_blink = !led_blink;
      blink_time = now + 500;
    }
}
