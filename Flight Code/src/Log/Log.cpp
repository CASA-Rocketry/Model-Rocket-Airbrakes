#include "Log.h"
#include "globalSettings.h"
#include "../hardwareMap.cpp"
#include <Arduino.h>

void Log::initialize(){
    pinMode(hardwareMap::SD_CD, INPUT);
    while(!hasCard()){
        sPrintln("Please Insert Card");
        delay(2000);
    }
    sPrintln("Card Detected");

}

bool Log::hasCard(){
    return digitalRead(hardwareMap::SD_CD) == LOW; //grounded when card in
}

