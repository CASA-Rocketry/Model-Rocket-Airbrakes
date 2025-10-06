//Log.ino includes both in-flight SD logging and debugging serial management since they utilitze the same framework

#include <SPI.h>
#include <SD.h>
#include <String>

#define SD_CS 10
//Logging variables for global scope convinience

String flightName, simName;
File flightFile, simFile;

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
  simFile = SD.open(simName);

    if(!simFile){
      Serial.println("ERROR opening flight file");
      enterErrorMode(5);
  } 
  //simFile.close();
}

void loadFlight(){
//Create flight file
  int flightNumber = 1;
  do{
    flightName = String((SIMULATION)? "s" : "") + "Flight" + String(flightNumber) + ".csv";
    Serial.println(flightName);
    flightNumber++;
  } while(SD.exists(flightName));
  
  flightFile = SD.open(flightName, FILE_WRITE);
  if(!flightFile){
    Serial.println("ERROR opening flight file");
    enterErrorMode(1);
  }
  Serial.println("Successfully logging to " + flightName);
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

