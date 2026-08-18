#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Clearwater sensor bring-up");
}

void loop() {
  // The first implementation target is a single DFRobot pH sensor.
}

