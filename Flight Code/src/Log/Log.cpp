#include "Log.h"
#include "../util/print.h"
#include "../hardware/hardwareMap.h"
#include <Arduino.h>
#include <string>
#include "../util/Config.hpp"
#include "../hardware/UI/UI.h"
#include <type_traits>

void Log::initialize(UI& ui){
    pinMode(hardwareMap::SD_CD, INPUT);

    //Check card detect - wait until recieved, don't go into error mode
    while(!hasCard()){
        ui.setTone(500, 1000);
        sPrint("Insert card -- ");
        delay(2000);
    }
    sPrintln("\nCard detected");

    //Start SPI communications with SD card
    if(!SD.begin(hardwareMap::SD_CS))
        ui.startError("Couldn't communicate with SD card");
    sPrintln("Successful log initialization");
}

void Log::printPreamble(std::string configString){
    //Log date and time of compile
    logPrintln(std::string("Code compiled on ") + __DATE__ + " at " + __TIME__);
    logPrintln(configString.c_str()); //Store config in logFile
    
    flushSD();
}

//Opens config file, reads all the data and sends to Config, then closes file
void Log::readConfig(Config& config, UI& ui){
    configFile = SD.open("config.csv", FILE_READ);
    std::string configString;
    if(configFile){
        sPrintln("Opened config successfully");
        while (configFile.available()){
            char newChar = configFile.read();
            configString += newChar; //Add next character
        }
        sPrintln("Finished reading config");
        config.configureConstants(configString);
    } else
        ui.startError("Config file not found");
    configFile.close();
}

bool Log::hasCard(){
    return digitalRead(hardwareMap::SD_CD) == LOW; //grounded when card in
}

void Log::openLogFile(std::string baseName, UI& ui){
    std::string flightFileName;
    int counter = 0;
    do{
        flightFileName = baseName + std::to_string(counter) + ".CSV";
        counter++;
    } while(SD.exists(flightFileName.c_str()));
    flightFile = SD.open(flightFileName.c_str(), FILE_WRITE);
    if(flightFile)
        sPrintln(("Successfully opened " + flightFileName + " for logging").c_str());
    else
        ui.startError("Could not open " + flightFileName + " for logging");
}


void Log::flushSD(){
    flightFile.flush();
}

//Calls all the getters and updates logLine
void Log::updateLogLine(){
    for(size_t i = 0; i < logGetters.size(); i++){
        logLine.at(i) = logGetters.at(i)();
    }
}

void Log::close(){
    flightFile.close();
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
    #if PRINT_IN_FLIGHT
        sPrint("LOG -- ");
        sPrintln(line.c_str());
    #endif
}