#include "PhysicalIMU.h"
#include "../../util/print.h"
#include "../UI/UI.h"

void PhysicalIMU::initialize(UI& ui){
    if(!orientationIMU.begin())
        ui.startError("Couldn't initialize orientation IMU");
    else if(!accelerationIMU.begin())
        ui.startError("Couldn't initialize orientation IMU");
    
    //Setup orientationIMU
    orientationIMU.setMode(OPERATION_MODE_IMUPLUS);
    orientationIMU.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
    orientationIMU.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);
    

    //Setup accelerationIMU
    accelerationIMU.setMode(OPERATION_MODE_CONFIG);
    accelerationIMU.write8(Adafruit_BNO055::adafruit_bno055_reg_t::BNO055_PAGE_ID_ADDR, 0x01);
    accelerationIMU.write8(Adafruit_BNO055::adafruit_bno055_reg_t::ACC_CONFIG, 0x13); //Write acc config (normal mode = 000, 125hz = 100, 16g range = 11)
    accelerationIMU.write8(Adafruit_BNO055::adafruit_bno055_reg_t::BNO055_PAGE_ID_ADDR, 0x00);

    accelerationIMU.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
    accelerationIMU.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);
    accelerationIMU.setMode(OPERATION_MODE_ACCONLY);

    adafruit_bno055_offsets_t orientationOffset{37, -51, 71, 35, -347, -224, -1, -2, -1, 1000, 1034}; //not using mag, but including anyway
    orientationIMU.setSensorOffsets(orientationOffset);

    adafruit_bno055_offsets_t accelerationOffset{13, -17, -24, 0, 0, 0, 0, 0, 4, 1000, 0}; //not using mag, but including anyway
    accelerationIMU.setSensorOffsets(accelerationOffset);

    dPrint("Orientation mode: "); dPrintln(orientationIMU.getMode());
    dPrint("Acceleration mode: "); dPrintln(accelerationIMU.getMode());

    //calibrate();

    sPrintln("IMU Initialized");
}

void PhysicalIMU::calibrate(){
    accelerationIMU.setMode(OPERATION_MODE_IMUPLUS);
    uint8_t calibrationStatus, otherStatus;

    //Gyro calibration (set still)
    sPrintln("Calibration gyro");
    do {
        accelerationIMU.getCalibration(&otherStatus, &calibrationStatus, &otherStatus, &otherStatus);
        sPrint(calibrationStatus);
        delay(100);
    } while(calibrationStatus != 3);
    sPrintln("Gyro calibration complete");

    sPrint("Calibrating accelerometer - ");

    do{
        accelerationIMU.getCalibration(&otherStatus, &otherStatus, &calibrationStatus, &otherStatus);
        sPrint(calibrationStatus);
        delay(100);
    } while(calibrationStatus != 3);

    //Magnetometer calibration wave around)
    sPrintln("Calibrating magnetometer");
    // do {
    //     accelerationIMU.getCalibration(&otherStatus, &otherStatus, &otherStatus, &calibrationStatus);
    //     delay(100);
    // } while(calibrationStatus != 3);
    sPrintln("Magnetometer calibration complete");

    //Accelerometer calibration (move for each axis to sense earth)
    


    sPrintln("\nAccelerometer calibration complete");
    adafruit_bno055_offsets_t offsets;
    accelerationIMU.getSensorOffsets(offsets);
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
    //dPrintln(rawLocalAcceleration.z());
    gravityLocalAcceleration = orientationIMU.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    localAcceleration = rawLocalAcceleration - gravityLocalAcceleration;
    globalAcceleration = quat.rotateVector(localAcceleration);
    //sPrintln(globalAcceleration.z());
}

//Assumes values have been read already
//Return in [0, pi]
double PhysicalIMU::getPitch(){
    return std::acos(1 - 2 * quat.x() * quat.x() - 2 * quat.y() * quat.y());
}