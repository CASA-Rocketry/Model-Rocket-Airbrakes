#include "PhysicalIMU.h"
#include "Log/print.h"
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
    UI::setTone(5000, 500);

    //Magnetometer calibration wave around)
    sPrintln("Calibrating magnetometer");
    do {
        bno.getCalibration(&otherStatus, &otherStatus, &otherStatus, &calibrationStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("magnetometer calibration complete");
    UI::setTone(5000, 500);
    delay(1000);
    UI::setTone(5000, 500);

    //Accelerometer calibration (move for each axis to sense earth)
    sPrintln("Calibrating accelerometer");
    // do{
    //     bno.getCalibration(&otherStatus, &otherStatus, &calibrationStatus, &otherStatus);
    //     delay(100);
    // } while(calibrationStatus != 3);
    sPrintln("Accelerometer calibration complete");
    UI::setTone(5000, 500);
    delay(1000);
    UI::setTone(5000, 500);
    delay(1000);
    UI::setTone(5000, 500);
}

PhysicalIMU::PhysicalIMU(){}

void PhysicalIMU::readAndCalculatePitch(){
    quat = bno.getQuat();
    //imu.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    localAcceleration = bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER) - bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    sPrintln(quat.x());
}