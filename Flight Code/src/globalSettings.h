#define SIM false
#define SERIAL true

#if SERIAL
  #define sPrint(a) Serial.print(a)
  #define sPrintln(a) Serial.println(a)
#else
  #define sPrint(a) //delete occurences of sPrint
  #define sPrintln(a) //delete occurence of sPrintln
#endif