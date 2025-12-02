#define SIM false
#define SERIAL_MODE true

#if SERIAL_MODE
  #define sPrint(a) Serial.print(a)
  #define sPrintln(a) Serial.println(a)
#else
  #define sPrint(a) //delete occurences of sPrint
  #define sPrintln(a) //delete occurence of sPrintln
#endif