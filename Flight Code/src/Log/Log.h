#include <SPI.h>
#include <SD.h>
#include "../hardwareMap.cpp"
#include <Arduino.h>
#include <string>
#pragma once

class Log{
private:
    File logFile, simFile, configFile;
public:
    void initialize();
    bool hasCard();
};