#include <String>

#define SIM_COLUMNS 21 //For Riley's sim

String simLine[SIM_COLUMNS] = {};

void initializeSim(){
  getNewLine(); //Clear header titles 
  getNewLine(); //Load first line
}

void updateSim(){
  getNewLine();
}

float getSimServoDeployment(){
  return simLine[6].toFloat();
}

float getSimPressure(){
  return 0;
}

float getSimAltitude(){
  return simLine[16].toFloat();
}

float getSimAccelerationZ(){
  return simLine[18].toFloat();
}

void getNewLine () {
  if (!flightFile.available())
    return; //end if no new data

  Serial.println("NEW LINE");
  char byte;
  String entry; //
  // while(flightFile.available()){
  //   byte = flightFile.read();
  //   Serial.println(byte);
  //   delay(100);
  // }

  for(int i = 0; i < SIM_COLUMNS; i++){ 
    entry = "";
    while(flightFile.available()){
      byte = flightFile.read();
      //if(byte == '\n') continue;
      if(byte == ',' || byte == '\n'){
        simLine[i] = entry;
        Serial.println(entry);
        break;
      } else
        entry += byte;
    }
  }
}
