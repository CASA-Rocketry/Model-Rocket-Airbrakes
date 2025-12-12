#include "PhysicalIMU.h"
#include "Log/print.h"

void PhysicalIMU::initialize(){
    //imu = Adafruit_BNO055()
    if(!imu.begin()){
        sPrintln("Error initializing IMU");
    }
}

PhysicalIMU::PhysicalIMU(){}

void PhysicalIMU::readAndCalculatePitch(){
    quat = imu.getQuat();
    localAcceleration = imu.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER);
}