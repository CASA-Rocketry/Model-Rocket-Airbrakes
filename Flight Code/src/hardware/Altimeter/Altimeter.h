#include "Adafruit_BMP3XX.h"
#include "../UI/UI.h"

//Abstract class to access physical and simulated altimeters
class Altimeter {
protected:
    
public:
    double altitude, temperature;
    virtual void readValues();
    virtual void initialize();
};

class PhysicalAltimeter{
private:
    Adafruit_BMP3XX bmp;
public:
    double altitude, temperature;
    double altitudeOffset;
    void calibrate(UI& ui);
    void readValues();
    void initialize(UI&);
};

class SimAltimeter : public Altimeter {
public:
    void readValues() override {}
    void initialize() override {}
};