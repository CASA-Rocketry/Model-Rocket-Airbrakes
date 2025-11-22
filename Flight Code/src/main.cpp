#include <Arduino.h>
#include "Rocket.h"

Rocket rocket;

void setup() {
  #if SERIAL
    Serial.begin(9600);
    while(!Serial);
  #endif
  rocket.initialize();
}

void loop() {
  rocket.readSensors();
  delay(100);
}

