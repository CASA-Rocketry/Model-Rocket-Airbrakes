#include <SPI.h>
#include <SD.h>
#include "../hardwareMap.h"
#include <Arduino.h>
#include <string>
#pragma once

const int VALUES_LOGGED = 10;

class Log{
private:
    File logFile, simFile, configFile;
    std::string logLine[VALUES_LOGGED] = {};
    std::function<std::string()> logGetters[VALUES_LOGGED] = {};
    int valuesAttached = 0; //tracks number of logLine entries that have been attached
    void readConfig();
    void openLogFile();
    void openSimFile();
    template <typename T>
    std::string toString(T);
public:
    void initialize();
    bool hasCard();
    template <typename T>
    void attachLogTag(std::string, T&);
};