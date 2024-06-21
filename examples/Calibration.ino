#include <Wire.h>
#include "Magnetometer.h"

Magnetometer mag;

void setup() {
  Serial.begin(38400);
  delay(5000);
  
  mag.begin();
  mag.calibrate();
  Serial.println("Magnetometer calibrated.");
}

void loop() {
  mag.update();
  Serial.print("X: ");
  Serial.print(mag.getX());
  Serial.print(" Y: ");
  Serial.print(mag.getY());
  Serial.print(" Z: ");
  Serial.println(mag.getZ());
  
  delay(500);
}