#include "Altimeter/Altimeter.h"
#include "Airbrake/Airbrake.h"
#include "Log/Log.h"
#include "IMU/PhysicalIMU.h"

class Rocket {
private:
    PhysicalAltimeter altimeter;
    Log log;
    PhysicalIMU imu;
    Airbrake brake;
    void addLogTags();
public:
    Rocket();
    ~Rocket();
    void readSensors();
    void update();
    void initialize();
};