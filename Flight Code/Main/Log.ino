//Log.ino includes both in-flight SD logging and debugging serial management since they utilitze the same framework

#include <SPI.h>
#include <SD.h>
#include <String>

#define SD_CS 10
//Logging variables for global scope convinience

String flightName, simName;
File flightFile, simFile;

void initializeLog(){
  

  #if SERIAL
    Serial.begin(9600);
    while(!Serial);
  #endif

  if(!SD.begin(SD_CS)){
    enterErrorMode("ERROR initialize log", 2);
  }

  #if SIMULATION
    loadSim();
  #endif
  loadFlight();

  //Create headers
  logLine[TIME_STAMP_LOG] = "time stamp";
  logLine[RAW_PRESSURE_LOG] = "raw pressure";
  logLine[RAW_TEMPERATURE_LOG] = "raw temperature";
  logLine[RAW_AX_LOG] = "raw ax";
  logLine[RAW_AY_LOG] = "raw ay";
  logLine[RAW_AZ_LOG] = "raw aZ";
  logLine[CALCULATED_ALT_LOG] = "calcualted alt";
  logLine[Y_ESTIMATE_LOG] = "y estimate";
  logLine[V_ESTIMATE_LOG] = "v estimate";
  logLine[A_ESTIMATE_LOG] = "a estimate";
  logLine[APOGEE_ESTIMATE_LOG] = "apogee estimate";
  logLine[SERVO_DEPLOYMENT_LOG] = "servo deployment";
  logLine[FLIGHT_MODE_LOG] = "flight mode";

  //add headers to log
  updateSD();
}

void loadSim(){
  simName = "SIM.csv";
  simFile = SD.open("SIM.csv");

    if(!simFile){
      enterErrorMode("ERROR opening sim file", 5);
  } 
}

void loadFlight(){
//Create flight file
  int flightNumber = 1;
  do{
    #if SIMULATION
      flightName = "simFlight" + String(flightNumber) + ".csv";
    #else
      flightName = "Flight" + String(flightNumber) + ".csv";
    #endif
    flightNumber++;
  } while(SD.exists(flightName));
  
  flightFile = SD.open(flightName, FILE_WRITE);
  if(!flightFile){
    enterErrorMode("ERROR opening flight file", 1);
  }
  sPrintln("Successfully logging to " + flightName);
  updateSD();
}

void endLog(){
  flightFile.close();
}

void updateSD(){
  for(int i = 0; i < ITEMS_LOGGED; i++){
    flightFile.print(logLine[i] + ',');
  }
  flightFile.print('\n');
}


void serialTag(String name, float value){
  Serial.print(name + ':');
  Serial.print(value);
  Serial.print(", ");
}