#include <SD.h>
#include <SPI.h>
#include <string.h>

#define SD_CS 10

File flightFile;

void initializeLog(){
  if(!SD.begin(SD_CS)){
    Serial.println("ERROR initializing log");
    enterErrorMode(2);
  }


//Create flight file
  File root = SD.open("/");

  int flightNumber = 1;
  while(root.openNextFile())
    flightNumber++;
  root.close();
  SD.remove("Flight.csv");
  Serial.println("Flight" + String(flightNumber) + ".csv");
  flightFile = SD.open("Flight.csv", FILE_WRITE);
  if(!flightFile){
    Serial.println("ERROR opening flight file");
    enterErrorMode(3);
  }
  flightFile.println("File opened");
}

void endLog(){
  flightFile.close();
}

void createLogHeader(String header){
  Serial.println("Time, Raw Altitude, Ky, Kv, Ka, y max predicted");
  flightFile.println("Time, Raw Altitude, Ky, Kv, Ka, y max predicted");
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


