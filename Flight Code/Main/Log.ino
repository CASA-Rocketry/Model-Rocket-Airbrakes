//Log.ino includes both in-flight SD logging and debugging serial management since they utilitze the same framework

#include <SPI.h>
#include <SD.h>
#include <String>

#define SD_CS 10
File flightFile;

void initializeLog(){
  if(SERIAL){
    Serial.begin(9600);
    //while(!Serial);
  }

  if(!SD.begin(SD_CS)){
    Serial.println("ERROR initializing log");
    enterErrorMode(2);
  }

  if (SIMULATION)
    loadSim();
  else
    loadFlight();
}

void loadSim(){
    flightFile = SD.open("SIM.csv");

    if(!flightFile){
      Serial.println("ERROR opening flight file");
      enterErrorMode(3);
  } 
}

void loadFlight(){
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
  updateSD();
}

void endLog(){
  flightFile.close();
}

void updateSD(){
  for(int i = 0; i < ITEMS_LOGGED; i++){
    flightFile.print(logLine[i] + ((i == ITEMS_LOGGED - 1) ? '\n' : ','));
  }
  flightFile.flush(); //Save new data to SD card
}


void serialTag(String name, float value){
  Serial.print(name + ':');
  Serial.print(value);
  Serial.print(", ");
}

