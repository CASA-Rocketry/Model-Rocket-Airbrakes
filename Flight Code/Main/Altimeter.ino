#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Pin constants
#define ALT_CS 6

//Parameter constnats
#define SEALEVELPRESSURE_HPA (1010.0)

#define CALIBRATION_POINT 0.0
#define CALIBRATION_SAMPLE_SIZE 100
#define CALIBRATION_SAMPLE_RATE 10 //ms

float offset;

Adafruit_BME280 alt(ALT_CS);

void initializeAlt(){
  unsigned status = alt.begin();
  if(!status){
    Serial.println("ERROR initializing alt");
    enterErrorMode(1);
  }
  calibrateAlt();
}

float getRawAlt(){
  return alt.readAltitude(SEALEVELPRESSURE_HPA);
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
      setLED(HIGH);
    else if(i % 10 == 0)
      setLED(LOW);

    delay(CALIBRATION_SAMPLE_RATE);
  }
  setLED(LOW);
    
  float average = sum / CALIBRATION_SAMPLE_SIZE;
  offset = CALIBRATION_POINT - average;
  Serial.print("Calibration complete! ... Average: ");
  Serial.println(average);
}

void logAltimeter(){
  logItem(getCalibratedAlt());
}

