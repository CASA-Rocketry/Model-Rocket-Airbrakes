#include "UI.h"
#include <Arduino.h>
#include "Log/print.h"
#include "../hardwareMap.h"
#include "constants.h"
#include "songs.hpp"

using namespace hardwareMap;

void UI::initialize(){
    sPrintln("Initializing UI");
    pinMode(hardwareMap::LED_RED, OUTPUT);
    pinMode(hardwareMap::LED_GREEN, OUTPUT);
    pinMode(hardwareMap::LED_BLUE, OUTPUT);

    pinMode(hardwareMap::BUTTON, INPUT);
    pinMode(hardwareMap::BUZZER, OUTPUT);

    pinMode(hardwareMap::BATT_TRANS, OUTPUT);
    digitalWrite(hardwareMap::BATT_TRANS, LOW);

    sPrintln("UI Initialized");
}

//Returns true for pressed, false for not pressed
bool UI::getButton(){
    //not pressed = LOW
    return digitalRead(hardwareMap::BUTTON) == HIGH;
}

void UI::setTone(int frequency, int duration){
    tone(hardwareMap::BUZZER, frequency, duration);
}

void UI::setRed(int value){
    digitalWrite(hardwareMap::LED_RED, value);
}

void UI::setGreen(int value){
    digitalWrite(hardwareMap::LED_GREEN, value);
}

void UI::setBlue(int value){
    digitalWrite(hardwareMap::LED_BLUE, value);
}

void UI::setColor(int r, int g, int b){
    setRed(r);
    setGreen(g);
    setBlue(b);
}

void UI::stopTone(){
    noTone(hardwareMap::BUZZER);
}

double UI::measureVoltage(){
    digitalWrite(hardwareMap::BATT_TRANS, HIGH);
    double rawVoltage = analogRead(hardwareMap::BATT_ANALOG);
    sPrintln(rawVoltage * constants::electrical::BATT_VOLTAGE_SCALER);
    
    //digitalWrite(hardwareMap::BATT_TRANS, LOW); 
    return 0;
}