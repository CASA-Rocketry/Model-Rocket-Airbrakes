#include "Altimeter.h"
#include "constants.h"
#include "Log/print.h"


void PhysicalAltimeter::initialize(){
    if(!bmp.begin_I2C()){
        Serial.println("ERROR initializing Altimeter"); 
    } else {
        bmp.setOutputDataRate(BMP3_ODR_200_HZ); 
        calibrate();
    }
}

//Updates temp and pressure data simulteneously 
void PhysicalAltimeter::readValues(){
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE) - altitudeOffset;
    temperature = bmp.temperature;
}

void PhysicalAltimeter::calibrate(){
    double sum = 0;
    altitudeOffset = 0; //get raw alt values

    for(int i = 0; i < CALIBRATION_POINTS; i++){
        readValues();
        sum += altitude;
        delay(CALIBRATION_DELAY);
    }
    altitudeOffset = sum / CALIBRATION_POINTS;
    sPrint("Calibration point: ");
    sPrintln(altitudeOffset);
}
