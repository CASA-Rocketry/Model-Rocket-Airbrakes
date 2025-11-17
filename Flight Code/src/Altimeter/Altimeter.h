#include "Adafruit_BMP3XX.h"

//Abstract class to access physical and simulated altimeters
class Altimeter {
public:
    virtual double getCalibratedAltitude() = 0;
    virtual double getTemperature() = 0;
    virtual void readValues() = 0;
};

class PhysicalAltimeter : public Altimeter {
private:
    Adafruit_BMP3XX bmp;
    double altitudeOffset;
    void calibrate();
public:
    void readValues();
    void initialize();
    double getTemperature();
    double getCalibratedAltitude();
};

class SimAltimeter : public Altimeter {
public:
    void readValues();
    double getTemperature();
    double getCalibratedAltitude();
};