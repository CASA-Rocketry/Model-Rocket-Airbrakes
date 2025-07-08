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
    delay(CALIBRATION_SAMPLE_RATE);
  }
    
  float average = sum / CALIBRATION_SAMPLE_SIZE;
  offset = CALIBRATION_POINT - average;
  Serial.print("Average: ");
  Serial.println(average);
}

void logAltimeter(){
  logItem(getCalibratedAlt());
}

