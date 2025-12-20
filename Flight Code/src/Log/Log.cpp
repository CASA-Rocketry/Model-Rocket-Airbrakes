#include "Log.h"
#include "print.h"
#include "hardware/hardwareMap.h"
#include <Arduino.h>
#include <string>
#include "../config.hpp"
#include <type_traits>

void Log::initialize(){
    sPrintln("Initializing log");
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

    printPreamble();
    
    if(config::SIMULATION);
        //openSimFile();
    sPrintln("Log initialized");
}

void Log::printPreamble(){
    //Log date and time of compile
    logPrintln(std::string("Code compiled on ") + __DATE__ + " at " + __TIME__);
    
    //Store config in logFile
    logPrintln(config::configString.c_str());
    flushSD();
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
    flightFile = SD.open(flightFileName.c_str(), FILE_WRITE);
    if(flightFile)
        sPrintln(("Successfully opened " + flightFileName + " for logging").c_str());
    else
        sPrintln(("Could not open " + flightFileName + " for logging").c_str());
}


void Log::flushSD(){
    flightFile.flush();
}

//Calls all the getters and updates logLine
void Log::updateLogLine(){
    for(int i = 0; i < logGetters.size(); i++){
        logLine.at(i) = logGetters.at(i)();
    }
}

//Writes logLine to SD card
void Log::writeLogLine(){
    std::string line = "";
    for(std::string str : logLine){
        line += str + ",";
    }
    logPrintln(line);
}

//Updates and writes logLine
void Log::update(){
    updateLogLine();
    writeLogLine();
}

void Log::attachTag(std::string name, std::function<std::string()> stringGetter){
    logLine.push_back(name);
    logGetters.push_back(stringGetter);
}

//Prints single line in log, no \n needed in string
void Log::logPrintln(std::string line){
    flightFile.write(line.c_str());
    flightFile.write("\n");
    #if PRINT_LOG_TO_SERIAL
        sPrint("LOG -- ");
        sPrintln(line.c_str());
    #endif
}