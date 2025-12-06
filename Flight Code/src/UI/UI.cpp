#include "UI.h"
#include <Arduino.h>
#include "globalSettings.h"
#include "../hardwareMap.h"

void UI::initialize(){
    pinMode(hardwareMap::LED_RED, OUTPUT);
    pinMode(hardwareMap::LED_GREEN, OUTPUT);
    pinMode(hardwareMap::LED_BLUE, OUTPUT);

    pinMode(hardwareMap::BUTTON, INPUT);
    pinMode(hardwareMap::BUZZER, OUTPUT);
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