#include "Log.h"
#include "globalSettings.h"
#include "../hardwareMap.h"
#include <Arduino.h>
#include <string>
#include "../config.h"

void Log::initialize(){
    pinMode(hardwareMap::SD_CD, INPUT);

    //Check card detect
    while(!hasCard()){
        sPrintln("Please insert card");
        delay(2000);
    }
    sPrintln("Card detected");

    //Start SPI communications with SD card
    if(!SD.begin(hardwareMap::SD_CS)){
        sPrintln("Couldn't communicate with SD card");
        return;
    }
    sPrintln("Able to read SD");

    readConfig();
    openLogFile();

    //Store config in logFile
    logFile.write(config::configString.c_str());
    logFile.flush();
    
    if(config::SIMULATION);
        //openSimFile();
}

//Opens config file, reads all the data and sends to Config, then closes file
void Log::readConfig(){
    configFile = SD.open("config.csv", FILE_READ);
        std::string configString;
        if(configFile){
            sPrintln("Opened config successfully");
            while (configFile.available()){
                char newChar = configFile.read();
                sPrint(newChar);
                configString += newChar; //Add next character
            }
            sPrintln(configString.c_str());
            config::configureConstants(configString);
        } else {
            sPrintln("Config file not found");
        }
    configFile.close();
}

bool Log::hasCard(){
    return digitalRead(hardwareMap::SD_CD) == LOW; //grounded when card in
}

void Log::openLogFile(){
    std::string baseName = config::LOG_NAME;
    sPrintln(baseName.c_str());
    std::string flightFileName;
    int counter = 0;
    do{
        flightFileName = baseName + std::to_string(counter) + ".CSV";
        sPrintln(flightFileName.c_str());
        counter++;
    } while(SD.exists(flightFileName.c_str()));
    logFile = SD.open(flightFileName.c_str(), FILE_WRITE);
    if(logFile)
        sPrintln(("Successfully opened " + flightFileName + " for logging").c_str());
    else
        sPrintln(("Could not open " + flightFileName + " for logging").c_str());
}