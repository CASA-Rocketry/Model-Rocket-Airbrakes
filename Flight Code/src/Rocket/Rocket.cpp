#include "Rocket.h"
#include "globalSettings.h"
#include "UI/UI.h"
#include "Log/Log.h"

Rocket::Rocket(){
    #if SIM
        altimeter = new SimAltimeter();
    #else
        altimeter = new PhysicalAltimeter();
    #endif
}

Rocket::~Rocket(){
    delete altimeter;
}

void Rocket::readSensors(){
    altimeter->readValues();
    //sPrintln(altimeter->getAltitude());
}

void Rocket::initialize(){
    altimeter->initialize();
    UI::initialize();
    UI::setTone(3000, 5000); //Buzzer test
    
    //Light test
    UI::setRed(1);
    delay(1000);
    UI::setGreen(1);
    delay(1000);
    UI::setBlue(1);
    delay(1000);
    UI::setColor(0, 0, 0);

    log.initialize();
}