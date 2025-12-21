#include "UI.h"
#include <Arduino.h>
#include "../../util/print.h"
#include "../hardwareMap.h"
#include "../../util/constants.h"
#include "songs.h"
#include "../../util/Config.hpp"
#include <string>

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

void UI::startError(std::string message, int code){
    sPrint("Fatal ERROR ------ ");
    sPrintln(message.c_str());
    setRed(HIGH);
    while(true){
        //Single long beep
        setTone(500, 2000);
        setBlue(HIGH);
        delay(2000);
        setBlue(LOW);
        delay(1000);

        //e.code shorter beeps
        for(int i = 0; i < code; i++){
            setTone(500, 500);
            setBlue(HIGH);
            delay(500);
            setBlue(LOW);
            delay(500);
        }
        delay(2000);
    }
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

void UI::playSong(int song[], int totalNotes, int tempo, int totalSeconds){
    //int totalNotes = sizeof(song) / sizeof(song[0]) / 2; //two entries per note
    int wholeNoteMillis = 60000 * 4 / tempo;
    int startMillis = millis(); 
    int noteIndex = 0; //0 to totalNotes
    int note, divider, noteDurationMillis;

    //Loop while time is less than 
    while(millis() - startMillis < totalSeconds * 1000){
        note = song[2 * noteIndex];
        divider = song[2 * noteIndex + 1];

        if(divider > 0)
            noteDurationMillis = wholeNoteMillis / divider;
        else
            noteDurationMillis = -1.5 * wholeNoteMillis / divider;
        setTone(note, noteDurationMillis * 0.9);
        delay(noteDurationMillis);
        stopTone();
        noteIndex = (noteIndex + 1) % totalNotes;
    }
}

void UI::playRandomSong(int totalSeconds, int seed){
    //Choose song number
    std::srand(seed);
    int songNumber = std::rand() % 5; 

    switch(songNumber){
        case 0:
            playSong(songs::hedwigsTheme, sizeof(songs::hedwigsTheme) / sizeof(songs::hedwigsTheme[0]) / 2, 144, totalSeconds);
            break;
        case 1: 
            playSong(songs::starWarsTheme, sizeof(songs::starWarsTheme) / sizeof(songs::starWarsTheme[0]) / 2, 108, totalSeconds);
            break;
        case 2: 
            playSong(songs::starWarsCantina, sizeof(songs::starWarsCantina) / sizeof(songs::starWarsCantina[0]) / 2, 250, totalSeconds);
            break;
        case 3: 
            playSong(songs::furElise, sizeof(songs::furElise) / sizeof(songs::furElise[0]) / 2, 80, totalSeconds);
            break;
        case 4: 
            playSong(songs::wiiTheme, sizeof(songs::wiiTheme) / sizeof(songs::wiiTheme[0]) / 2, 114, totalSeconds);
            break;  
    }
}