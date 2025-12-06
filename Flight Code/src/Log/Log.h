#include <SPI.h>
#include <SD.h>
#include "../hardwareMap.h"
#include <Arduino.h>
#include <string>
#pragma once

class Log{
private:
    File logFile, simFile, configFile;
    void readConfig();
    void openLogFile();
    void openSimFile();
public:
    void initialize();
    bool hasCard();
};