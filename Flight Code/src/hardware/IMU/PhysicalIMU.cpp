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
    
    //adafruit_bno055_offsets_t orientationOffset; //not using mag, but including anyway
    orientationIMU.setSensorOffsets({26, -42, 78, -11, 58, -2, -2, 0, 0, 1000, 1000});

    //adafruit_bno055_offsets_t accelerationOffset{34, -41, 75, 467, -55, 25, -1, -2, 1, 1000, 1000}; //not using mag, but including anyway

    //Setup accelerationIMU
    accelerationIMU.setMode(OPERATION_MODE_CONFIG);
    accelerationIMU.write8(Adafruit_BNO055::adafruit_bno055_reg_t::BNO055_PAGE_ID_ADDR, 0x01);
    delay(20); //Probably not needed
    accelerationIMU.write8(Adafruit_BNO055::adafruit_bno055_reg_t::ACC_CONFIG, 0x13); //Write acc config (normal mode = 000, 125hz = 100, 16g range = 11)
    delay(20); //Probably not needed
    accelerationIMU.write8(Adafruit_BNO055::adafruit_bno055_reg_t::BNO055_PAGE_ID_ADDR, 0x00);
    delay(20); //Probably not needed
    accelerationIMU.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
    accelerationIMU.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);
    accelerationIMU.setMode(OPERATION_MODE_ACCONLY);
    
    
    AInverse.cell(0, 0) = 0.985079;
    AInverse.cell(0, 1) = -0.028604;
    AInverse.cell(0, 2) = 0.003267;
    AInverse.cell(1, 0) = -0.028604;
    AInverse.cell(1, 1) = 0.990997;
    AInverse.cell(1, 2) = 0.006620;
    AInverse.cell(2, 0) = 0.003267;
    AInverse.cell(2, 1) = 0.006620;
    AInverse.cell(2, 2) = 0.996102;

    bias(0) = 0.057610;
    bias(1) = -0.209385;
    bias(2) = 0.707400;


    #if DEBUG
        while(true){
            readValues();
            sPrintln(rawLocalAcceleration.z());
            delay(100);
        }
    #endif


    dPrint("Orientation mode: "); dPrintln(orientationIMU.getMode());
    dPrint("Acceleration mode: "); dPrintln(accelerationIMU.getMode());

    
    //calibrate();

    sPrintln("IMU Initialized");
}

void PhysicalIMU::calibrateOrientationIMU(UI& ui){
    sPrintln("Calibrating orientation IMU");
    while(!ui.getButton()){
        printAcceleration(orientationIMU);
        sPrint(" ----- ");
        printCalibration(orientationIMU);
        sPrintln();
        delay(100);
    }
    printOffsets(orientationIMU);
}

void PhysicalIMU::calibrateAccelerationIMU(UI& ui){
    while(true){
        if(ui.getButton()){
            delay(500); //half second to allow it to stabilize after button press 
            imu::Vector<3> averageAcceleration;
            const int DATA_POINTS = 100;
            for(int i = 0; i < DATA_POINTS; i++){
                readValues();
                averageAcceleration = averageAcceleration + rawLocalAcceleration;
                delay(50);
            }
            averageAcceleration = averageAcceleration / DATA_POINTS;

            //Print to console
            sPrint(averageAcceleration.x()); sPrint("\t"); 
            sPrint(averageAcceleration.y()); sPrint("\t"); 
            sPrint(averageAcceleration.z()); sPrint("\n");

            ui.setTone(5000, 500);
        } else
            delay(100);
    }
}

void PhysicalIMU::printAcceleration(Adafruit_BNO055& bno){
    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER);
    sPrint("("); sPrint(acceleration.x()); sPrint(","); sPrint(acceleration.y()); sPrint(","); sPrint(acceleration.z()); sPrint(")");
}

void PhysicalIMU::printCalibration(Adafruit_BNO055& bno){
    uint8_t systemCalib, gyroCalib, magCalib, accCalib;
    bno.getCalibration(&systemCalib, &gyroCalib, &accCalib, &magCalib);
    sPrint("system: "); sPrint(systemCalib); sPrint(", gyro: "); sPrint(gyroCalib); sPrint(", acc: "); sPrint(accCalib); sPrint(", mag: "); sPrint(magCalib);
}

void PhysicalIMU::printOffsets(Adafruit_BNO055& bno){
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);

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
  
  sPrint("\n\n\n");
  sPrint("{"); 
  sPrint(offsets.accel_offset_x); sPrint(", "); 
  sPrint(offsets.accel_offset_y); sPrint(", "); 
  sPrint(offsets.accel_offset_z); sPrint(", "); 
  sPrint(offsets.mag_offset_x); sPrint(", "); 
  sPrint(offsets.mag_offset_y); sPrint(", "); 
  sPrint(offsets.mag_offset_z); sPrint(", ");
  sPrint(offsets.gyro_offset_x); sPrint(", ");
  sPrint(offsets.gyro_offset_y); sPrint(", ");
  sPrint(offsets.gyro_offset_z); sPrint(", "); 
  sPrint(offsets.accel_radius); sPrint(", "); 
  sPrint(offsets.accel_radius); sPrintln("}");
}

PhysicalIMU::PhysicalIMU(){}

void PhysicalIMU::readValues(){
    quat = orientationIMU.getQuat();

    //Apply accerleration calibration
    imu::Matrix<3> unscaledLocalAcceleration;
    unscaledLocalAcceleration.vector_to_col(accelerationIMU.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER)-bias, 0);
    
    rawLocalAcceleration = (AInverse * unscaledLocalAcceleration).col_to_vector(0);



    gravityLocalAcceleration = orientationIMU.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GRAVITY);
    localAcceleration = rawLocalAcceleration - gravityLocalAcceleration;
    globalAcceleration = quat.rotateVector(localAcceleration);
}

//Assumes values have been read already
//Return in [0, pi]
double PhysicalIMU::getPitch(){
    return std::acos(1 - 2 * quat.x() * quat.x() - 2 * quat.y() * quat.y());
}