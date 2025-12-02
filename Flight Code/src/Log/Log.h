#include <SPI.h>
#include <SD.h>
#include "../hardwareMap.cpp"
#include <Arduino.h>
#pragma once

class Log{
private:
    File file;
public:
    void initialize();
    bool hasCard();
};