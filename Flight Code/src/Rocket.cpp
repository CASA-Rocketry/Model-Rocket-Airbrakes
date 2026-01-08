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

    //UI Button management
    buttonPrevious = false;
    usButtonStart = 0;
}

Rocket::~Rocket(){
}

void Rocket::readSensors(){
    altimeter.readValues();
    imu.readValues();
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

    //imu.calibrate();
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
    sPrint("Tip rocket over for 5 seconds to begin altimeter lockout of ");  sPrintln(config.ALTIMETER_LOCKOUT_SECONDS);
    Trigger::reset();
    while(Trigger::getHoldState(ui.getButton(), 5000)) //require 5 second hold
        delay(50);
    Trigger::reset();
    ui.playRandomSong(config.ALTIMETER_LOCKOUT_SECONDS, millis());
    altimeter.calibrate();
    ui.setTone(5000, 5000);
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
    //log.attachTag("Estimated apogee", [&] () -> double {return control::getApogee(stateEstimator.y(), stateEstimator.v(), )})

    

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
    //brake.setDeployment(control::computeDeployment(stateEstimator.y(), stateEstimator.v(), config));

    //Check for apogee (outside of modes in case of stating error)
    if(stateEstimator.y() > apogeeMeters){
        apogeeMeters = stateEstimator.y();
        usApogee = usCurrent;
    }


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
        case COASTING: //Also includes first several seconds of recovery so as to not prematurely switch modes
            brake.setDeployment(control::computeDeployment(stateEstimator.y(), stateEstimator.v(), config));
            if(stateEstimator.y() < 20 && stateEstimator.v() < -0.5)
                mode = RECOVERY;
            break;
        case RECOVERY:
            log.flushSD();
            brake.setDeployment(0);
            if(std::abs(stateEstimator.v()) < 0.1){
                mode = LANDED;
                usLand = usCurrent;
            }
            break;
    }

    //Allow ending regardless of state, though it should occur in LANDED mode
    //5 second continuous hold to 
    if(Trigger::getHoldState(ui.getButton(), 5000))
        end();

    log.update();
}

void Rocket::end(){
    ui.setTone(500, 3000);
    ui.setColor(1, 1, 1);
    log.logPrintln("Apogee was " + std::to_string(apogeeMeters) + " and occured at " + std::to_string((usApogee - usLaunch)/1000000.0) + " seconds");
    log.logPrintln("Total flight time was " + std::to_string((usLand - usLaunch)/1000000.0) + " seconds");
    log.close();
    
    //Continuosly (5s period) print summary info to Serial, even if not connected
    while(true){
        printTag("Apogee (m)", apogeeMeters);
        printTag("Apogee time stamp (s)", (usApogee - usLaunch)/1000000.0);
        printTag("Total flight time (s)", (usLand - usLaunch)/1000000.0);
        delay(5000);
    }
}