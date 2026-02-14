#include "Altimeter.h"
#include "../../util/constants.h"
#include "../../util/print.h"


void PhysicalAltimeter::initialize(UI& ui){
    if(!bmp.begin_I2C())
        ui.startError("Can't communicate with BMP");
    bmp.setOutputDataRate(BMP3_ODR_200_HZ); 
    sPrintln("Altimeter initialized");
}

//Updates temp and pressure data simulteneously 
void PhysicalAltimeter::readValues(){
    altitude = bmp.readAltitude(constants::physics::SEA_LEVEL_PRESSURE) - altitudeOffset; //Single reading performed here
    temperature = bmp.temperature;
}

void PhysicalAltimeter::calibrate(UI& ui){
    double sum = 0;
    altitudeOffset = 0; //get raw alt values
    const double CALIBRATION_POINTS = 100;
    bmp.performReading(); //flush out first reading, generally bad

    for(int i = 0; i < CALIBRATION_POINTS; i++){
        ui.setTone(4000, 50);
        readValues();
        sum += altitude;
        delay(100);
    }
    altitudeOffset = sum / CALIBRATION_POINTS;
    sPrintln("Altimeter calibrated");
}
