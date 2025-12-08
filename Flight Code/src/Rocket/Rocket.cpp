#include "Rocket.h"
#include "globalSettings.h"
#include "UI/UI.h"
#include "Log/Log.h"


Rocket::Rocket(){

}

Rocket::~Rocket(){
}

void Rocket::readSensors(){
    //altimeter.readValues();
    //sPrintln(altimeter->getAltitude());
}


void Rocket::initialize(){
    //altimeter->initialize();
    int x = 0;
    bool b = false;
    UI::initialize();


    
    // brake.test();
    log.initialize();
    log.attachTag("Test", x);
    log.attachTag(" TF Test", b);
    log.writeLogLine();
    log.update();
    x = 2;
    b = true;
    log.update();
    

    //Indicate successful initialization
    UI::setTone(3000, 5000);
    
    //Light test
    UI::setRed(1);
    delay(1000);
    UI::setGreen(1);
    delay(1000);
    UI::setBlue(1);
    delay(1000);
    UI::setColor(0, 0, 0);
}