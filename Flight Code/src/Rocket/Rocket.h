#include "Altimeter/Altimeter.h"
#include "Log/Log.h"

class Rocket {
private:
    Altimeter* altimeter;
    Log log;
public:
    Rocket();
    ~Rocket();
    void readSensors();
    void updateLog();
    void initialize();
};