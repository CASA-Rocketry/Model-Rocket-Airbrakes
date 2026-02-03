#include "Rocket.h"
#include "hardware/UI/UI.h"
#include "Log/Log.h"
#include "util/print.h"
#include "control/control.h"
#include <cmath>
#if DEBUG
    #include "util/Timer.h"
#endif

#define WIND_TUNNEL false


Rocket::Rocket(){
    //Main time tracking
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

    //Must tip rocket to at least 135 degrees for 5 seconds
    do {
        imu.readValues();
        delay(50);
        dPrintln(imu.getPitch() * 180 / M_PI);
    } while (!Trigger::getHoldState(imu.getPitch() > 0.75 * M_PI, 5000));
    Trigger::reset();

    #if !DEBUG
        ui.playRandomSong(config.ALTIMETER_LOCKOUT_SECONDS, millis());
    #endif
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

    log.attachTag("Commanded servo deployment", brake.commandedDeployment);
    log.attachTag("Real servo deployment", brake.currentDeployment);

    //Timing
    log.attachTag("dt (us) (last cycle)", usDelta);
    //log.attachTag("Process times (us)", Timer::logLine);
    log.attachTag("Estimated apogee", [&] () -> std::string {return Timer::logLine;});

    

    //Print headers
    log.writeLogLine();
    log.flushSD();
}

void Rocket::update(){
    //Update times
    usLast = usCurrent;
    usCurrent = micros();
    usDelta = usCurrent - usLast;
    if(usDelta > 20000){
        dPrint("Loop overrun: "); dPrintln(std::to_string(usDelta).c_str());
    }

    Timer::resetTime();
    readSensors(); //Inicludes calculation of all sensor quantities prior to KF fusion
    Timer::endProcess("Read sensors");

    Timer::resetTime();
    stateEstimator.update(altimeter.altitude, imu.globalAcceleration.z(), usDelta / 1000000.0);
    Timer::endProcess("State estimator update");

    //Check for apogee (outside of modes in case of stating error)
    if(stateEstimator.y() > apogeeMeters){
        apogeeMeters = stateEstimator.y();
        usApogee = usCurrent;
    }

    #if WIND_TUNNEL
        updateWindTunnel();
    #else
        updateFlightStates();
    #endif

    //Allow ending regardless of state, though it should occur in LANDED mode
    //5 second continuous hold to 
    if(Trigger::getHoldState(ui.getButton(), 5000))
        end();

    Timer::resetTime();
    log.update(); 
    Timer::resetLogLine(); //Clear time stamp log for this cycle
    Timer::endProcess("Log (last cycle)"); //Time for THIS cycle while be included in line for NEXT cycle
}

void Rocket::updateFlightStates(){
    switch(mode){
            case SETUP:
                ui.startError("update() called without rocket initialization", 4);
                break;
            case IDLE:
                if(imu.globalAcceleration.z() >= config.LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED){
                    mode = BURNING;
                    usLaunch = usCurrent;
                    dPrintln("IDLE -> BURNIGN");
                }
                #if DEBUG
                    mode = BURNING;
                    usLaunch = usCurrent;
                    dPrintln("IDLE -> BURNIGN");
                #endif

                break; //Wait cycle to start
            case BURNING:
                if(usCurrent - usLaunch > config.COAST_LOCKOUT_SECONDS * 1000 * 1000){
                    mode = COASTING;
                    control::startRateLimiter(config); //starts rate limiter at 0
                    dPrintln("BURING -> COASTING");
                }
                break;
            case COASTING: //Also includes first several seconds of recovery so as to not prematurely switch modes
                brake.setDeployment(control::computeDeployment(stateEstimator.y(), stateEstimator.v(), config));
                    #if !DEBUG
                    if(stateEstimator.y() < 20 && stateEstimator.v() < -0.5)
                        mode = RECOVERY;
                    #endif
                break;
            case RECOVERY:
                log.flushSD();
                brake.setDeployment(0);
                if(std::abs(stateEstimator.v()) < 0.1){
                    mode = LANDED;
                    usLand = usCurrent;
                }
                break;
            case LANDED:
                break;
        }
}

void Rocket::updateWindTunnel(){
    //on for 30s, off for 30s, repeat
        if(usCurrent % (1000*1000*100) <= 1000*1000*20)
            brake.setDeployment(0);
        else if (usCurrent % (1000*1000*100) <= 1000*1000*40)
            brake.setDeployment(0.25);
        else if (usCurrent % (1000*1000*100) <= 1000*1000*60)
            brake.setDeployment(0.5);
        else if (usCurrent % (1000*1000*100) <= 1000*1000*80)
            brake.setDeployment(0.75);
        else
            brake.setDeployment(1);
}

void Rocket::end(){
    ui.setTone(500, 3000);
    ui.setColor(1, 1, 1);
    log.logPrintln("Apogee was " + std::to_string(apogeeMeters) + " and occured at " + std::to_string((usApogee - usLaunch)/1000000.0) + " seconds");
    log.logPrintln("Total flight time was " + std::to_string((usLand - usLaunch)/1000000.0) + " seconds");
    log.close();
    
    //Continuosly (5s period) print summary info to Serial, even if not connected
    while(true){
        sPrintTag("Apogee (m)", apogeeMeters);
        sPrintTag("Apogee time stamp (s)", (usApogee - usLaunch)/1000000.0);
        sPrintTag("Total flight time (s)", (usLand - usLaunch)/1000000.0);
        delay(5000);
    }
}