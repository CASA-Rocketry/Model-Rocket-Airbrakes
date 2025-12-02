#include "Altimeter/Altimeter.h"
#include "Airbrake/Airbrake.h"
#include "Log/Log.h"

class Rocket {
private:
    //PhysicalAltimeter altimeter;
    Log log;
    Airbrake brake;
public:
    Rocket();
    ~Rocket();
    void readSensors();
    void updateLog();
    void initialize();
};