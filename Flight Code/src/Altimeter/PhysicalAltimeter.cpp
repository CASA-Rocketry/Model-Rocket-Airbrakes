#include "Altimeter.h"
#include "Constants.cpp"


void PhysicalAltimeter::initialize(){
    if(!bmp.begin_I2C())
        Serial.println("ERROR initializing Altimeter");
    else 
        calibrate();
}

//Updates temp and pressure data simulteneously 
//NOT USED as readAltitude and readTemperature redudantly grab the data right now
void PhysicalAltimeter::readValues(){
    bmp.performReading();
}

void PhysicalAltimeter::calibrate(){
    double sum = 0;
    for(int i = 0; i < CALIBRATION_POINTS; i++){
        sum += bmp.readAltitude(SEA_LEVEL_PRESSURE);
        delay(20);
    }
    altitudeOffset = sum / CALIBRATION_POINTS;
}

double PhysicalAltimeter::getCalibratedAltitude(){
    return bmp.readAltitude(SEA_LEVEL_PRESSURE) - altitudeOffset;
}

double PhysicalAltimeter::getTemperature(){
    return bmp.readTemperature();
}