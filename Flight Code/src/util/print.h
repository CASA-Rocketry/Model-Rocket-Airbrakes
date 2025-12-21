#pragma once

#include <Arduino.h>

//These configuration modes could be moved to config, though macros should be more efficient
#define SERIAL_ENABLED true
#define WAIT_FOR_SERIAL_CONNECTION true
#define PRINT_IN_FLIGHT true //determines whether to print during flight (ie. for testing)

#if SERIAL_ENABLED
  #define sPrint(a) Serial.print(a)
  #define sPrintln(a) Serial.println(a)
  #define printTag(a, b) sPrint(a); sPrint(": "); sPrintln(b)
#else
  #define sPrint(a) //delete occurences of sPrint
  #define sPrintln(a) //delete occurence of sPrintln
  #define printTag(a, b) //delete occurences of printTag
#endif
