#include "Adafruit_BMP3XX.h"

//Abstract class to access physical and simulated altimeters
class Altimeter {
protected:
    
public:
    double altitude, temperature;
    virtual void readValues();
    virtual void initialize();
};

class PhysicalAltimeter : public Altimeter {
private:
    Adafruit_BMP3XX bmp;
public:
    double altitudeOffset;
    void calibrate();
    void readValues() override;
    void initialize() override;
};

class SimAltimeter : public Altimeter {
public:
    void readValues() override {}
    void initialize() override {}
};