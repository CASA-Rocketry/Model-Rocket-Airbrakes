#include <Arduino_LSM9DS1.h>

void initializeIMU(){
  if(!IMU.begin()){
    Serial.println("ERROR Initializing IMU");
    enterErrorMode(4);
  }
}