#include "hardware/Altimeter/Altimeter.h"
#include "hardware/Airbrake/Airbrake.h"
#include "Log/Log.h"
#include "hardware/IMU/PhysicalIMU.h"
#include "Config.hpp"
#include "hardware/UI/UI.h"

class Rocket {
private:
    Config config;
    PhysicalAltimeter altimeter;
    Log log;
    PhysicalIMU imu;
    Airbrake brake;
    UI ui;
    void addLogTags();
public:
    Rocket();
    ~Rocket();
    void readSensors();
    void update();
    void setup();
};