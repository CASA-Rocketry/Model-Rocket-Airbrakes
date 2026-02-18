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

    #if DEBUG
        usFrom30 = 0;
    #endif

    //Secondary time stamps
    usLaunch = usApogee = usLand = 0;
    apogeeMeters = 0;
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
    //ui.setTone(4000, 1000); //Turn on beep

    //Log startup, read config, and preamble
    log.initialize(ui);
    log.readConfig(config, ui);
    sPrintln("Parsed Configuration ------------------------------------------------");
    config.printCheck();
    sPrintln("---------------------------------------------------------------------");
    log.openLogFile(config.LOG_NAME, ui);
    log.printPreamble(config.configString);

    //stateEstimator.fillFromConfig(config);
    
    // altimeter.initialize(ui);
    // imu.initialize(ui);
    // brake.initialize();
    // if(config.AIRBRAKES_ENABLED)
    //     brake.enable();
    // brake.test();

    ui.setTone(4000, 3000); //3s high tone for completion
    sPrintln("Initialization COMPLETE --------------------------------------------");

    // if(config.SIMULATION){
    //     sPrintln("Running in SIMULATION");
    //     ui.setBlue(1);
    // } else {
    //     sPrintln("Running in FLIGHT mode");
    //     ui.setGreen(1);
    // }
    // sPrint("Tip rocket over for 5 seconds to begin altimeter lockout of ");  sPrintln(config.ALTIMETER_LOCKOUT_SECONDS);
    // Trigger::reset();

    // //Must tip rocket to at least 135 degrees for 5 seconds
    // do {
    //     imu.readValues();
    //     delay(50);
    //    // sPrintln(imu.getPitch() * 180 / M_PI);
    // } while (!Trigger::getHoldState(imu.getPitch() > 0.75 * M_PI, 5000));
    // Trigger::reset();

    #if !DEBUG
        ui.playRandomSong(config.ALTIMETER_LOCKOUT_SECONDS, millis());
    #endif
    // altimeter.calibrate(ui);
    // ui.setTone(4000, 5000); //5 sec success beep
    // log.logPrintln("Calibration point: " + std::to_string(altimeter.altitudeOffset));
    
    addLogTags();
    mode = IDLE;
}

void Rocket::addLogTags(){
    log.attachTag("Time (us)", usCurrent);

    //Timing
    #if DEBUG
        // log.attachTag("dt (us) (last cycle)", usDelta);
        // log.attachTag("Process times (us)", Timer::logLine);
        // log.attachTag("Estimated apogee", [&] () -> std::string {return Timer::logLine;});
    #endif

    //Print headers
    log.writeLogLine();
    log.flushSD();

   
}

void Rocket::update(){
    //Update times
    usLast = usCurrent;
    usCurrent = micros();
    usDelta = usCurrent - usLast;

    log.update(); 

    if(ui.getButton()) //Single button press to end test
        end();
}

void Rocket::updateFlightStates(){
    switch(mode){
            case SETUP:
                ui.startError("update() called without rocket initialization");
                break;
            case IDLE:
                if(imu.globalAcceleration.z() >= config.LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED){
                    #if !DEBUG 
                        mode = BURNING; 
                        ui.setTone(500); //We should never here this 
                    #endif
                    usLaunch = usCurrent;
                    //dPrintln("IDLE -> BURNING");
                }
                break; //Wait cycle to start
            case BURNING:
                if(usCurrent - usLaunch > config.COAST_LOCKOUT_SECONDS * 1000 * 1000){
                    #if !DEBUG
                        mode = COASTING;
                        
                    #endif
                    control::startRateLimiter(config); //starts rate limiter at 0
                    //dPrintln("BURING -> COASTING");
                }
                break;
            case COASTING: //Also includes first several seconds of recovery so as to not prematurely switch modes
                brake.setDeployment(control::computeDeployment(stateEstimator.y(), stateEstimator.v(), config));
                // if(usCurrent - usLaunch > 1000 * 1000 * 2.5)
                //     brake.setDeployment(1); 
                // else 
                //     brake.setDeployment(0);   
                #if !DEBUG
                    if(stateEstimator.y() < 20 && stateEstimator.v() < -0.5)
                        mode = RECOVERY;
                #endif
                break;
            case RECOVERY:
                log.flushSD();
                brake.setDeployment(0);
                if(std::abs(stateEstimator.v()) < 0.1){
                    #if !DEBUG
                        mode = LANDED;
                    #endif
                    usLand = usCurrent;
                }
                break;
            case LANDED:
                break;
        }
}

#if DEBUG
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

void Rocket::updateFlightStateForDebug(){
    //Change flight states every 30 seconds
    if(usCurrent % (30*1000*1000) < usFrom30){
        //Advance flight mode by 1
        dPrintln("Switching mode");
        switch(mode){
            case IDLE:
                mode = BURNING;
                break;
            case BURNING:
                mode = COASTING;
                break;
            case COASTING:
                mode = RECOVERY;
                break;
            case RECOVERY:
                mode = LANDED;
                break;
            case LANDED:
                mode = IDLE;
                break;
        }
    }

    usFrom30 = usCurrent % (30 * 1000 * 1000);
}
#endif

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