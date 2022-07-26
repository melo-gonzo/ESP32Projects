#include <Arduino.h>

// ledPin refers to ESP32-CAM GPIO 4 (flashlight)
const int ledPin = 4;

void setup()
{
  Serial.begin(115200);
  // initialize digital pin ledPin as an output
  pinMode(ledPin, OUTPUT);
  Serial.print("Setup");
}

void loop()
{
  digitalWrite(ledPin, HIGH);
  Serial.print("High");
  delay(2000);
  digitalWrite(ledPin, LOW);
  Serial.print("Low");
  delay(2000);
}