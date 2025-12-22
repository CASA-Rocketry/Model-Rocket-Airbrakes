#include <Adafruit_BNO055.h>
#include <string>

class PhysicalIMU{
private:    
    Adafruit_BNO055 bno;
    
public: 
    PhysicalIMU();
    void initialize();
    void getPitch();
    void readValues();
    void calibrate();
    double pitch; //radians
    imu::Quaternion quat;
    imu::Vector<3> localAcceleration, globalAcceleration;
};