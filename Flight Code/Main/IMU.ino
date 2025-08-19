#include "Arduino_BMI270_BMM150.h"
float gx, gy, gz, ax, ay, az, mx, my, mz;



void updateIMU(){
  if(IMU.accelerationAvailable())
    IMU.readAcceleration(az, ax, ay); //Redefined reference frame
  
  logLine[2] = String(ax);
  logLine[3] = String(ay);
  logLine[4] = String(az);

}

void initializeIMU(){
  if(!IMU.begin()){
    Serial.println("ERROR Initializing IMU");
    enterErrorMode(4);
  }
}

void updateAcceleration(){
  

}

//Assumes rocket is in vertical flight
float getVerticalAcceleration(){
  return 9.8 * (az - 1.0);
}

float getDragAcceleration(){
  return 0;//g * sqrt(ax*ax + ay*ay + az*az);
}