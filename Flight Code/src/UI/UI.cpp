#include "UI.h"
#include <Arduino.h>
#include "globalSettings.h"
#include "../hardwareMap.cpp"

void UI::initialize(){
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    pinMode(BUTTON, INPUT);
    pinMode(BUZZER, OUTPUT);
    sPrintln("UI Initialized");
}

//Returns true for pressed, false for not pressed
bool UI::getButton(){
    //not pressed = LOW
    return digitalRead(BUTTON) == HIGH;
}

void UI::setTone(int frequency, int duration){
    tone(BUZZER, frequency, duration);
}

void UI::setRed(int value){
    digitalWrite(LED_RED, value);
}

void UI::setGreen(int value){
    digitalWrite(LED_GREEN, value);
}

void UI::setBlue(int value){
    digitalWrite(LED_BLUE, value);
}

void UI::setColor(int r, int g, int b){
    setRed(r);
    setGreen(g);
    setBlue(b);
}