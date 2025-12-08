#include <Arduino.h>
#include "Rocket/Rocket.h"
#include "globalSettings.h"

Rocket rocket;

void setup() {
  #if SERIAL_MODE
    Serial.begin(9600);
    //while(!Serial);
  #endif

  rocket.initialize();
}

void loop() {
  rocket.readSensors();
  //sPrintln("Hello");
  delay(100);
}

