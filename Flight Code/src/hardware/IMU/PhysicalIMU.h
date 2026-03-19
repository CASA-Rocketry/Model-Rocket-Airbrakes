#include <Adafruit_BNO055.h>
#include <string>
#include "../UI/UI.h"
#include <BasicLinearAlgebra.h>
#include "../../util/print.h"

class PhysicalIMU{
private:    
    Adafruit_BNO055 orientationIMU{-1, 0x28}; //Maybe switch
    Adafruit_BNO055 accelerationIMU{-1, 0x29};

    #if DEBUG
    void printCalibration(Adafruit_BNO055&);
    void printOffsets(Adafruit_BNO055&);
    void printAcceleration(Adafruit_BNO055&);
    void calibrateAccelerationIMU(UI&);
    void calibrateOrientationIMU(UI&);
    #endif

    //Calibration constants
    imu::Matrix<3> AInverse;
    imu::Vector<3> bias;

    imu::Matrix<3> unscaledLocalAcceleration; //3 x 3 matrix with first column containing bais compensated raw local acceleration measurements. 
    //terrible way to get matrix x vector multiplication to work with this library, but workable)
    
public: 
    PhysicalIMU();
    void initialize(UI&);
    void readValues();
    void calibrate();
    double getPitch();
    double getCosPitch();
    //double pitch; //radians
    imu::Quaternion quat;
    imu::Vector<3> rawLocalAcceleration, gravityLocalAcceleration, localAcceleration, globalAcceleration;
};