# Ultrasonic

```cpp
class Ultrasonic {
  public:
  Ultrasonic(int echoPin, int trigPin): echoPin(echoPin), trigPin(trigPin) {
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  }

  float ping() {
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * time2cm; // Speed of sound wave divided by 2 (go and back)
    return distance;
  }

  float distance;

  protected:
  const int echoPin, trigPin;
  static constexpr float time2cm = 0.034 / 2; // 100 cm/m 340 m/s 1E-6 s/us x duration us
};

Ultrasonic us(2,3);

void setup() {
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}
void loop() {
  us.ping();

  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(us.distance);
  Serial.println(" cm");

  delay(500);
}
```