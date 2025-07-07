#include <SD.h>
#include <SPI.h>
#include <string.h>

#define SD_CS 10

File flightFile;

void initializeLog(){
  if(!SD.begin(SD_CS)){
    Serial.println("ERROR initializing log");
  }


//Create flight file
  File root = SD.open("/");

  int flightNumber = 1;
  while(root.openNextFile())
    flightNumber++;
  root.close();

  Serial.println("Flight" + String(flightNumber) + ".txt");
  flightFile = SD.open("Flight" + String(flightNumber) + ".csv", FILE_WRITE);
  if(!flightFile){
    Serial.println("ERROR opening flight file");
  }
  flightFile.println("File opened");
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


