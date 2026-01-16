#include <Adafruit_BNO055.h>
#include <string>
#include "../UI/UI.h"

class PhysicalIMU{
private:    
    Adafruit_BNO055 orientationIMU{0x28};
    Adafruit_BNO055 accelerationIMU{0x29};
    
public: 
    PhysicalIMU();
    void initialize();
    void readValues();
    void calibrate();
    double getPitch();
    //double pitch; //radians
    imu::Quaternion quat;
    imu::Vector<3> rawLocalAcceleration, gravityLocalAcceleration, localAcceleration, globalAcceleration;
};