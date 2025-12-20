#include "Rocket.h"
#include "UI/UI.h"
#include "Log/Log.h"
#include "Log/print.h"


Rocket::Rocket(){

}

Rocket::~Rocket(){
}

void Rocket::readSensors(){
    altimeter.readValues();
    imu.readAndCalculatePitch();
    //sPrintln(altimeter->getAltitude());
}


void Rocket::initialize(){
    //altimeter->initialize();
    //altimeter.initialize();
    //imu.initialize();
    UI::initialize();
    
    sPrintln("initialize");
    while(true){
        UI::measureVoltage();
        sPrintln("test");
        delay(500);
    }
    // while(true)
    //     delay(1000);

    
    // brake.test();
    log.initialize();
    // addLogTags();

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

    altimeter.calibrate();
}

void Rocket::addLogTags(){
    log.attachTag("Altitude AGL (m)", altimeter.altitude);
    log.attachTag("Temperature (deg C)", altimeter.temperature);

    log.attachTag("IMU Quat W", imu.quat.w());
    log.attachTag("IMU Quat X", imu.quat.x());
    log.attachTag("IMU Quat Y", imu.quat.y());
    log.attachTag("IMU Quat Z", imu.quat.z());
    log.attachTag("IMU Local Acceleration X", imu.localAcceleration.x());
    log.attachTag("IMU Local Acceleration Y", imu.localAcceleration.y());
    log.attachTag("IMU Local Acceleration Z", imu.localAcceleration.z());
    log.attachTag("Pitch (radians)", imu.pitch);
    //Print headers
    log.writeLogLine();
}

void Rocket::update(){
    readSensors();
    log.update();
    if(UI::getButton()){
        log.flushSD();
        sPrintln("Flushing SD");
    }
}