#include "Altimeter/Altimeter.h"

class Rocket {
private:
    Altimeter* altimeter;
public:
    Rocket();
    ~Rocket();
    void readSensors();
    void updateLog();
    void initialize();
};