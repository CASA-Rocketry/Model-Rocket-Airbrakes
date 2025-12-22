#include "hardware/Altimeter/Altimeter.h"
#include "hardware/Airbrake/Airbrake.h"
#include "Log/Log.h"
#include "hardware/IMU/PhysicalIMU.h"
#include "util/Config.hpp"
#include "hardware/UI/UI.h"
#include "control/StateEstimator.h"

class Rocket {
private:
    enum Mode {
        SETUP = 0,
        IDLE = 1,
        BURNING = 2,
        COASTING = 3,
        RECOVERY = 4, //Could split into multiple states if running time control
        LANDED = 5
    } mode;
    //Maing time tracking (us stands for micro seconds = 1/10^6 seconds)
    unsigned long usCurrent, usLast, usDelta; //usngiend long gives 2^32 / 10000 / 60 / 60 = 1.19 hours of run time

    //Secondary time stamps
    unsigned long usLaunch, usApogee, usLand;
    double apogeeMeters;

    //UI Button management
    bool buttonPrevious;
    unsigned long usButtonStart;

    
    StateEstimator stateEstimator;
    Config config;
    PhysicalAltimeter altimeter;
    Log log;
    PhysicalIMU imu;
    Airbrake brake;
    UI ui;
    void addLogTags();
    void readSensors();
    void end();
public:
    Rocket();
    ~Rocket();

    void update();
    void setup();
};