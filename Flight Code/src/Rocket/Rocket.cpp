#include "Rocket.h"
#include "hardware/UI/UI.h"
#include "Log/Log.h"
#include "Log/print.h"
#include "exceptions.h"
#include "control.h"


Rocket::Rocket(){

}

Rocket::~Rocket(){
}

void Rocket::readSensors(){
    altimeter.readValues();
    imu.readAndCalculatePitch();
}


void Rocket::setup(){
    log.initialize(config);
    ui.initialize();
    altimeter.initialize();
    imu.initialize();

    imu.calibrate();
    brake.test();

    ui.playRandomSong(config.ALTIMETER_LOCKOUT_SECONDS, millis());
    altimeter.calibrate();
    

    // addLogTags();
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
    if(ui.getButton()){
        log.flushSD();
        sPrintln("Flushing SD");
    }
}