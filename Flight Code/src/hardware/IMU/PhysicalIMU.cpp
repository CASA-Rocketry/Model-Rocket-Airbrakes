#include "PhysicalIMU.h"
#include "../../util/print.h"
#include "../UI/UI.h"

void PhysicalIMU::initialize(){
    sPrintln("Initializing IMU");
    if(!orientationIMU.begin())
        sPrintln("Error initializing orietnationIMU");
    else if(!accelerationIMU.begin())
        sPrintln("Error initializing accelerationIMU");
    
    //Setup orientationIMU
    orientationIMU.setMode(OPERATION_MODE_IMUPLUS);
    orientationIMU.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
    orientationIMU.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);

    //Setup accelerationIMU
    accelerationIMU.setMode(OPERATION_MODE_ACCONLY);
    accelerationIMU.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
    accelerationIMU.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);
    //TODO: ensure high output frequency (~200 hz)


    //adafruit_bno055_offsets_t offsets{3, -12, -34, 35, -347, -224, 0, -2, -1, 1000, 1034};
    //bno.setSensorOffsets(offsets);

    dPrint("Orientation mode: "); dPrintln(orientationIMU.getMode());
    dPrint("Acceleration mode: "); dPrintln(accelerationIMU.getMode());

    sPrintln("IMU Initialized");
}

void PhysicalIMU::calibrate(){
    //bno.setMode(OPERATION_MODE_NDOF);
    uint8_t calibrationStatus, otherStatus;

    //Gyro calibration (set still)
    sPrintln("Calibration gyro");
    do {
        orientationIMU.getCalibration(&otherStatus, &calibrationStatus, &otherStatus, &otherStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("Gyro calibration complete");

    sPrint("Calibrating accelerometer - ");
    do{
        orientationIMU.getCalibration(&otherStatus, &otherStatus, &calibrationStatus, &otherStatus);
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
    orientationIMU.getSensorOffsets(offsets);
    while(true){
        sPrint("Accelerometer: ");
        sPrint(offsets.accel_offset_x); sPrint(" ");
        sPrint(offsets.accel_offset_y); sPrint(" ");
        sPrint(offsets.accel_offset_z); sPrint(" ");

        sPrint("\nGyro: ");
        sPrint(offsets.gyro_offset_x); sPrint(" ");
        sPrint(offsets.gyro_offset_y); sPrint(" ");
        sPrint(offsets.gyro_offset_z); sPrint(" ");

        sPrint("\nMag: ");
        sPrint(offsets.mag_offset_x); sPrint(" ");
        sPrint(offsets.mag_offset_y); sPrint(" ");
        sPrint(offsets.mag_offset_z); sPrint(" ");

        sPrint("\nAccel Radius: ");
        sPrint(offsets.accel_radius);

        sPrint("\nMag Radius: ");
        sPrint(offsets.mag_radius);
        delay(1000);
    }
    // bno.setMode(OPERATION_MODE_IMUPLUS); //reset mode to not include magnetometer
}

PhysicalIMU::PhysicalIMU(){}

void PhysicalIMU::readValues(){
    quat = orientationIMU.getQuat();
    //dPrintln(accelerationIMU.getMode());
    rawLocalAcceleration = accelerationIMU.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER);
    gravityLocalAcceleration = orientationIMU.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    localAcceleration = rawLocalAcceleration - gravityLocalAcceleration;
    globalAcceleration = quat.rotateVector(localAcceleration);
}

//Assumes values have been read already
//Return in [0, pi]
double PhysicalIMU::getPitch(){
    return std::acos(1 - 2 * quat.x() * quat.x() - 2 * quat.y() * quat.y());
}