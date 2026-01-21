#pragma once

#include <Arduino.h>

//These configuration modes could be moved to config, though macros should be more efficient
#define SERIAL_ENABLED true
#define WAIT_FOR_SERIAL_CONNECTION true
#define PRINT_IN_FLIGHT false //determines whether to print during flight (ie. for testing)
#define DEBUG true //ie. remove before flight if false

#if SERIAL_ENABLED 
  #define sPrint(a) Serial.print(a)
  #define sPrintln(a) Serial.println(a)
  #define sPrintTag(a, b) sPrint(a); sPrint(": "); sPrintln(b)
#else
  #define sPrint(a) //delete occurences of sPrint
  #define sPrintln(a) //delete occurence of sPrintln
  #define sPrintTag(a, b) //delete occurences of printTag
#endif

#if DEBUG
  #define dPrint(a) sPrint(a)
  #define dPrintln(a) sPrintln(a)
#else
  #define dPrint(a)
  #define dPrintln(a)
#endif
