#include <SPI.h>
#include <SD.h>
#include "../hardwareMap.h"
#include <Arduino.h>
#include <string>
#include <vector>
#pragma once


class Log{
private:
    File logFile, simFile, configFile;
    std::vector<std::string> logLine;
    std::vector<std::function<std::string()>> logGetters;
    int valuesAttached = 0; //tracks number of logLine entries that have been attached
    void readConfig();
    void openLogFile();
    void openSimFile();
    void updateLogLine();

public:
    void initialize();
    bool hasCard();

    //Templated methods need to be defined in .h file
    template <typename T> void attachTag(std::string name, T& valRef){
        //Add name to first logLine (header line)
        logLine.push_back(name);
        std::function<std::string()> stringGetter;
        if(std::is_same<T, bool>::value){
            //Bool to string
            stringGetter = [&]() {return valRef ? "T" : "F";};
        } else {
            //Int/double/float to string
            stringGetter = [&]() {return std::to_string(valRef);};
        }
        logGetters.push_back(stringGetter);
    }
    
    void update();
    void flushSD();
    void writeLogLine(); //Could be private, but used once publically to write headers
};