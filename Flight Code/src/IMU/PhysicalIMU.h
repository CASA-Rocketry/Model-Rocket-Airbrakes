#include <Adafruit_BNO055.h>
#include <string>

class PhysicalIMU{
private:    
    Adafruit_BNO055 imu;
    
public: 
    PhysicalIMU();
    void initialize();
    void getPitch();
    void readAndCalculatePitch();
    double pitch; //radians
    imu::Quaternion quat;
    imu::Vector<3> localAcceleration;
};