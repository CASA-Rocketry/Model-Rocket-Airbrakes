#include <Arduino.h>
#include "Rocket/Rocket.h"
#include "Log/print.h"

Rocket rocket;

void setup() {
  startSerial();
  rocket.initialize();
}

void loop() {
  rocket.update();
  //sPrintln("Hello");
  delay(100);
}

