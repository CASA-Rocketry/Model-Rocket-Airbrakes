#include "Rocket.h"
#include "hardware/UI/UI.h"
#include "Log/Log.h"
#include "util/print.h"
#include "control/control.h"


Rocket::Rocket(){

}

Rocket::~Rocket(){
}

void Rocket::readSensors(){
    altimeter.readValues();
    imu.readAndCalculatePitch();
}


void Rocket::setup(){
    sPrintln("Starting rocket setup");
    //All subsystem-specific print messages are included within their respective method
    ui.initialize();

    //Log startup, read config, and preamble
    log.initialize(ui);
    log.readConfig(config, ui);
    sPrintln("--------------------------------------------------- Raw Configuration");
    sPrintln(config.configString.c_str());
    sPrintln("------------------------------------------------ Parsed Configuration");
    config.printCheck();
    sPrintln("---------------------------------------------------------------------");
    log.openLogFile(config.LOG_NAME, ui);
    log.printPreamble(config.configString);
    
    // if(config.SIMULATION);
    //     //openSimFile();
    altimeter.initialize();
    imu.initialize();
    brake.initialize();
    if(config.AIRBRAKES_ENABLED)
        brake.enable();

    imu.calibrate();
    brake.test();

    ui.setTone(2000, 5000);
    sPrintln("Initialization COMPLETE");
    ui.setColor(0, 0, 0);
    if(config.SIMULATION){
        sPrintln("Running in SIMULATION");
        ui.setBlue(1);
    } else {
        sPrintln("Running in FLIGHT mode");
        ui.setGreen(1);
    }
    sPrint("Press button to start altimeter lockout of ");  sPrintln(config.ALTIMETER_LOCKOUT_SECONDS);
    while(!ui.getButton())
        delay(50);
    ui.playRandomSong(config.ALTIMETER_LOCKOUT_SECONDS, millis());
    altimeter.calibrate();
    log.logPrintln("Calibration point: " + std::to_string(altimeter.altitudeOffset));
    
    addLogTags();
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
    log.flushSD();
}

void Rocket::update(){
    readSensors();
    log.update();
    if(ui.getButton()){
        log.flushSD();
        ui.setBlue(1);
    } else  
        ui.setBlue(0);
}