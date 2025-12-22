#include "Rocket.h"
#include "hardware/UI/UI.h"
#include "Log/Log.h"
#include "util/print.h"
#include "control/control.h"


Rocket::Rocket(){
    //Maing time tracking
    usCurrent = usLast = usDelta = 0;

    //Secondary time stamps
    usLaunch = usApogee = usLand = 0;
    apogeeMeters = 0;
}

Rocket::~Rocket(){
}

void Rocket::readSensors(){
    altimeter.readValues();
    imu.readAndCalculatePitch();
}


void Rocket::setup(){
    sPrintln("Starting rocket setup");
    mode = SETUP;

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

    stateEstimator.fillFromConfig(config);
    
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
    mode = IDLE;
}

void Rocket::addLogTags(){
    log.attachTag("Time (us)", usCurrent);
    log.attachTag("Altitude AGL (m)", altimeter.altitude);
    log.attachTag("Temperature (deg C)", altimeter.temperature);
    log.attachTag("Mode", mode);

    log.attachTag("IMU Quat W", imu.quat.w());
    log.attachTag("IMU Quat X", imu.quat.x());
    log.attachTag("IMU Quat Y", imu.quat.y());
    log.attachTag("IMU Quat Z", imu.quat.z());
    log.attachTag("IMU Local Acceleration X", imu.localAcceleration.x());
    log.attachTag("IMU Local Acceleration Y", imu.localAcceleration.y());
    log.attachTag("IMU Local Acceleration Z", imu.localAcceleration.z());
    log.attachTag("IMU Global Acceleration x", imu.globalAcceleration.x());
    log.attachTag("IMU Global Acceleration y", imu.globalAcceleration.y());
    log.attachTag("IMU Global Acceleration z", imu.globalAcceleration.z());

    log.attachTag("State Estimation y", stateEstimator.y());
    log.attachTag("State Estimation v", stateEstimator.v());
    log.attachTag("State Estimation a", stateEstimator.a());

    log.attachTag("Servo deployment", brake.deployment);

    

    //Print headers
    log.writeLogLine();
    log.flushSD();
}

void Rocket::update(){
    //Update times
    usLast = usCurrent;
    usCurrent = micros();
    usDelta = usCurrent - usLast;

    readSensors(); //Inicludes calculation of all sensor quantities prior to KF fusion
    stateEstimator.update(altimeter.altitude, imu.globalAcceleration.z(), usDelta / 1000000.0);

    switch(mode){
        case SETUP:
            ui.startError("update() called without rocket initialization", 4);
            break;
        case IDLE:
            if(imu.globalAcceleration.z() >= config.LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED){
                mode = BURNING;
                usLaunch = usCurrent;
            }
            break; //Wait cycle to start
        case BURNING:
            if(usCurrent - usLaunch > config.COAST_LOCKOUT_SECONDS * 1000 * 1000)
                mode = COASTING;
            break;
        case COASTING: //Also includes first several seconds of recovery
            //if()
            break;
        
    }


    log.update();

    //10 second continuious press stops program
    if(ui.getButton()){
        log.flushSD();
        ui.setBlue(1);
        ui.startError("Program finished", 7);
    } else  
        ui.setBlue(0);
}