#include "PhysicalIMU.h"
#include "../../util/print.h"
#include "../UI/UI.h"

void PhysicalIMU::initialize(){
    sPrintln("Initializing IMU");
    if(!bno.begin()){
        sPrintln("Error initializing IMU");
    }
    
    bno.setMode(OPERATION_MODE_IMUPLUS);
    bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);
    //adafruit_bno055_offsets_t offsets{3, -12, -34, 35, -347, -224, 0, -2, -1, 1000, 1034};
    //bno.setSensorOffsets(offsets);

    sPrintln("IMU Initialized");
}

void PhysicalIMU::calibrate(){
    //bno.setMode(OPERATION_MODE_NDOF);
    uint8_t calibrationStatus, otherStatus;

    //Gyro calibration (set still)
    sPrintln("Calibration gyro");
    do {
        bno.getCalibration(&otherStatus, &calibrationStatus, &otherStatus, &otherStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("Gyro calibration complete");

    sPrint("Calibrating accelerometer - ");
    do{
        bno.getCalibration(&otherStatus, &otherStatus, &calibrationStatus, &otherStatus);
        sPrint(calibrationStatus);
        delay(100);
    } while(calibrationStatus != 3);

    //Magnetometer calibration wave around)
    sPrintln("Calibrating magnetometer");
    do {
        //bno.getCalibration(&otherStatus, &otherStatus, &otherStatus, &calibrationStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("Magnetometer calibration complete");

    //Accelerometer calibration (move for each axis to sense earth)
    


    sPrintln("\nAccelerometer calibration complete");
    adafruit_bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);
    while(true){
    Serial.print("Accelerometer: ");
    Serial.print(offsets.accel_offset_x); Serial.print(" ");
    Serial.print(offsets.accel_offset_y); Serial.print(" ");
    Serial.print(offsets.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(offsets.gyro_offset_x); Serial.print(" ");
    Serial.print(offsets.gyro_offset_y); Serial.print(" ");
    Serial.print(offsets.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(offsets.mag_offset_x); Serial.print(" ");
    Serial.print(offsets.mag_offset_y); Serial.print(" ");
    Serial.print(offsets.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(offsets.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(offsets.mag_radius);
    delay(1000);
    }
    // bno.setMode(OPERATION_MODE_IMUPLUS); //reset mode to not include magnetometer
}

PhysicalIMU::PhysicalIMU(){}

void PhysicalIMU::readValues(){
    quat = bno.getQuat();
    //imu.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    localAcceleration = bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER) - bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    globalAcceleration = quat.rotateVector(localAcceleration);
}

//Assumes values have been read already
//Return in [0, pi]
double PhysicalIMU::getPitch(){
    return std::acos(1 - 2 * quat.x() * quat.x() - 2 * quat.y() * quat.y());
}