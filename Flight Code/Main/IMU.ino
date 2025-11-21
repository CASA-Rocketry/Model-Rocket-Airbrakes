#include "src/Libraries/Arduino_BMI270_BMM150/Arduino_BMI270_BMM150.h"
float gx, gy, gz, ax, ay, az, mx, my, mz;



void updateIMU(){
  #if SIMULATION
    ax = simLine[3].toFloat();
    ay = simLine[4].toFloat();
    az = simLine[5].toFloat();
  #else
    if(IMU.accelerationAvailable())
      IMU.readAcceleration(ay, az, ax); //Redefined reference frame
    
    az *= -1;
  #endif
  
  logLine[3] = String(ax);
  logLine[4] = String(ay);
  logLine[5] = String(az);
}

void initializeIMU(){
  if(!IMU.begin()){
    Serial.println("ERROR Initializing IMU");
    enterErrorMode(4);
  }
  IMU.setContinuousMode();
}

//Assumes rocket is in vertical flight
float getVerticalAcceleration(){
  return 9.8 * (az - 1.0);
}

float getDragAcceleration(){
  return 0;//g * sqrt(ax*ax + ay*ay + az*az);
}