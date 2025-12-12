#pragma once

#include <Arduino.h>

//These configuration modes could be moved to config, though macros should be more efficient
#define SERIAL_ENABLED true
#define WAIT_FOR_SERIAL_CONNECTION true
#define PRINT_LOG_TO_SERIAL true //determines whether log data should also be printed to serial for debugging

#if SERIAL_ENABLED
  #define sPrint(a) Serial.print(a)
  #define sPrintln(a) Serial.println(a)
#else
  #define sPrint(a) //delete occurences of sPrint
  #define sPrintln(a) //delete occurence of sPrintln
#endif

void startSerial(){
    #if SERIAL_ENABLED
        Serial.begin(9600);
        #if WAIT_FOR_SERIAL_CONNECTION
            while(!Serial);
        #endif
    #endif
}