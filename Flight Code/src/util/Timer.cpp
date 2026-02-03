#include "Timer.h"
#include <Arduino.h>

namespace Timer{
unsigned long startTime = 0;
std::string logLine = "";

void resetTime(){
    startTime = micros();
}

void endProcess(std::string processName){
    unsigned long dt = micros() - startTime;
    logLine += processName + ":" + std::to_string(dt) + "|";
}

void resetLogLine(){
    logLine = "";
}
}