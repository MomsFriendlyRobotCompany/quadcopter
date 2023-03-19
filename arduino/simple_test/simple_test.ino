#include <TFmini.h>
#include <Adafruit_NeoPixel.h>
// #include <common/mavlink.h>
#include <mavlink2.h>

constexpr int LED_PIN = 18;
constexpr int LED_COUNT = 1; // How many NeoPixels are attached to the Arduino?

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

TFmini tfmini;

void setup() {
    Serial.begin(1000000);
    Serial.setTimeout(5);
    while (!Serial)
        delay(10);

    
    Serial1.begin(TFmini::DEFAULT_BAUDRATE);
    tfmini.attach(Serial1);

    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();

}

void loop() {
  Serial.println("hello");

  pixels.setPixelColor(0, pixels.Color(0, 0, 100));
  pixels.show();

  if (tfmini.available()) {
    Serial.println("tfmini ");
    Serial.print("distance : ");
    Serial.println(tfmini.getDistance());
    Serial.print("strength : ");
    Serial.println(tfmini.getStrength());
    Serial.print("int time : ");
    Serial.println(tfmini.getIntegrationTime());
  }


  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    1,
    100,
    &msg,
    MAV_TYPE_QUADROTOR,
    MAV_AUTOPILOT_GENERIC,
    MAV_MODE_GUIDED_ARMED,
    0,
    MAV_STATE_ACTIVE);  // MAV_STATE_STANDBY
  
  delay(100);
}
