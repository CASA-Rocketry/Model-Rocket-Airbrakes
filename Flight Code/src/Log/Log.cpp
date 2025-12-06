#include "Log.h"
#include "globalSettings.h"
#include "../hardwareMap.cpp"
#include <Arduino.h>
#include <string>
#include "../config.h"

void Log::initialize(){
    pinMode(hardwareMap::SD_CD, INPUT);
    while(!hasCard()){
        sPrintln("Please insert card");
        delay(2000);
    }

    if(!SD.begin(hardwareMap::SD_CS)){
        sPrintln("Couldn't communicate with SD card");
    } else {
        sPrintln("Card detected");
        configFile = SD.open("config.csv", FILE_READ);
        std::string config;
        if(configFile){
            while (configFile.available()){
                char newChar = configFile.read();
                sPrint(newChar);
                config += newChar; //Add next character
            }
            sPrintln(config.c_str());
            configureConstants(config);
        } else {
            sPrintln("Config file not found");
        }
    }
}

bool Log::hasCard(){
    return digitalRead(hardwareMap::SD_CD) == LOW; //grounded when card in
}

