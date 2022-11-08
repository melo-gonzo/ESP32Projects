#include <Arduino.h>

// ledPin refers to ESP32-CAM mini led on back. Use GPIO pin 4 for flashlight
const int ledPin = 33;

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