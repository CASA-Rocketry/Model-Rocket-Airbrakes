#include <Arduino_LPS22HB.h>

//Parameter constnats
#define SEALEVEL_PRESSURE_KPA 101.325

#define CALIBRATION_POINT 0.0
#define CALIBRATION_SAMPLE_SIZE 100
#define CALIBRATION_SAMPLE_RATE 20 //ms

float offset;

void initializeAlt(){
  if(!BARO.begin()){
    Serial.println("ERROR Initilizing Altimeter");
    enterErrorMode(1);
  }

  calibrateAlt();
}


float getRawAlt(){
  float pressure = BARO.readPressure();
  return 44330 * ( 1 - pow(pressure/SEALEVEL_PRESSURE_KPA, 1/5.255)); //TODO: update with temperature information
}


float getCalibratedAlt(){
  return getRawAlt() + offset;
}

void calibrateAlt(){
  float sum = 0;
  for(int i = 0; i < CALIBRATION_SAMPLE_SIZE; i++){
    sum += getRawAlt();
    //LED blinking
    if(i % 10 == 5)
      setGreenLED(HIGH);
    else if(i % 10 == 0)
      setGreenLED(LOW);

    delay(CALIBRATION_SAMPLE_RATE);
  }
  setGreenLED(LOW);
    
  float average = sum / CALIBRATION_SAMPLE_SIZE;
  offset = CALIBRATION_POINT - average;
  Serial.print("Calibration complete! ... Average: ");
  Serial.println(average);
}

void logAltimeter(){
  logItem(getCalibratedAlt());
}

