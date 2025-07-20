#include "Arduino_BMI270_BMM150.h"

#define g 9.8

float a_x = 0;
float a_y = 0;
float a_z = 0;

void initializeIMU(){
  if(!IMU.begin()){
    Serial.println("ERROR Initializing IMU");
    enterErrorMode(4);
  }
}

void updateAcceleration(){
  if(IMU.accelerationAvailable()){
    IMU.readAcceleration(a_z, a_x, a_y); //Redefined reference frame
  }
}

//Assumes rocket is in vertical flight
float getVerticalAcceleration(){
  return g * (a_z - 1);
}

float getDragAcceleration(){
  return g * sqrt(a_x*a_x + a_y*a_y + a_z*a_z);
}