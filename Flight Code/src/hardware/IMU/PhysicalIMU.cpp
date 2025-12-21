#include "PhysicalIMU.h"
#include "../../util/print.h"
#include "../UI/UI.h"

void PhysicalIMU::initialize(){
    sPrintln("Initializing IMU");
    if(!bno.begin()){
        sPrintln("Error initializing IMU");
    }
    sPrintln("IMU Initialized");
}

void PhysicalIMU::calibrate(){
    uint8_t calibrationStatus, otherStatus;

    //Gyro calibration (set still)
    sPrintln("Calibration gyro");
    do {
        bno.getCalibration(&otherStatus, &calibrationStatus, &otherStatus, &otherStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("Gyro calibration complete");

    //Magnetometer calibration wave around)
    sPrintln("Calibrating magnetometer");
    do {
        bno.getCalibration(&otherStatus, &otherStatus, &otherStatus, &calibrationStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("Magnetometer calibration complete");

    //Accelerometer calibration (move for each axis to sense earth)
    sPrint("Calibrating accelerometer - ");
    do{
        calibrationStatus = 3;
        //bno.getCalibration(&otherStatus, &otherStatus, &calibrationStatus, &otherStatus);
        sPrint(calibrationStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("\nAccelerometer calibration complete");
}

PhysicalIMU::PhysicalIMU(){}

void PhysicalIMU::readAndCalculatePitch(){
    quat = bno.getQuat();
    //imu.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    localAcceleration = bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER) - bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
}