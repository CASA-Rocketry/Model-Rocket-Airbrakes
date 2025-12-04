#include "Log.h"
#include "globalSettings.h"

void Log::initialize(){
    pinMode(SD_CD, INPUT);
    sPrintln(hasCard());
}

bool Log::hasCard(){
    return digitalRead(SD_CD) == LOW; //grounded when card in
}

