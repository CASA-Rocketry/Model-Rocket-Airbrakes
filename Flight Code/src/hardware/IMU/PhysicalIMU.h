#include <Adafruit_BNO055.h>
#include <string>
#include "../UI/UI.h"
#include <BasicLinearAlgebra.h>

class PhysicalIMU{
private:    
    Adafruit_BNO055 orientationIMU{-1, 0x28}; //Maybe switch
    Adafruit_BNO055 accelerationIMU{-1, 0x29};

    void printCalibration(Adafruit_BNO055&);
    void printOffsets(Adafruit_BNO055&);
    void printAcceleration(Adafruit_BNO055&);
    void calibrateAccelerationIMU(UI&);
    void calibrateOrientationIMU(UI&);

    //Calibration constants
    imu::Matrix<3> AInverse;
    imu::Vector<3> bias;
    
public: 
    PhysicalIMU();
    void initialize(UI&);
    void readValues();
    void calibrate();
    double getPitch();
    //double pitch; //radians
    imu::Quaternion quat;
    imu::Vector<3> rawLocalAcceleration, gravityLocalAcceleration, localAcceleration, globalAcceleration;
};