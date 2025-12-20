#include <Arduino.h>
#include "Rocket/Rocket.h"
#include "Log/print.h"

Rocket rocket;

void setup() {
  #if SERIAL_ENABLED
      Serial.begin(9600);
      #if WAIT_FOR_SERIAL_CONNECTION
          while(!Serial);
      #endif
  #endif
  rocket.setup();
}

void loop() {
  rocket.update();
}

