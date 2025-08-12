//Log.ino includes both in-flight SD logging and debugging serial management since they utilitze the same framework

#include <SPI.h>
#include <SD.h>
#include <string.h>

#define SD_CS 10

File flightFile;

void initializeLog(){
  if(DEBUG){
    Serial.begin(9600);
    while(!Serial);
  }

  if(!SD.begin(SD_CS)){
    Serial.println("ERROR initializing log");
    enterErrorMode(2);
  }

//Create flight file
  int flightNumber = 1;
  String fileName;
  do{
    fileName = "Flight" + String(flightNumber) + ".csv";
    flightNumber++;
  } while(SD.exists(fileName));
  

  flightFile = SD.open(fileName, FILE_WRITE);
  if(!flightFile){
    Serial.println("ERROR opening flight file");
    enterErrorMode(3);
  }
  Serial.println("Successfully logging to " + fileName);
  flightFile.print("Time, Raw Altitude, Ky, Kv, Ka, y max predicted");
}

void endLog(){
  flightFile.close();
}

void logItem(double value){
  flightFile.print(value);
  flightFile.print(',');
}

void newLogLine(){
  flightFile.flush(); //Save new data to SD card
  flightFile.print('\n');
  logItem(millis()/1000.0);
}

void updateLogs(){
  newLogLine();
  logAltimeter();
  logStateEstimation();
}

void serialTag(String name, float value){
  Serial.print(name + ':');
  Serial.print(value);
  Serial.print(", ");
}

