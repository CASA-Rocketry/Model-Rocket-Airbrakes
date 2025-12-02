#include <String>

void initializeSim(){
  getNewLine(); //Clear header titles 
  //getNewLine(); //Load first line
}

void updateSim(){
  //flightFile.close();
  //flightFile = SD.open(simName, FILE_READ);
  getNewLine();
  //flightFile.close();
  //flightFile = SD.open(flightName, FILE_WRITE);
}

void getNewLine () {
  if (!simFile.available())
    return; //end if no new data

  //Serial.println("NEW LINE");
  char byte;
  String entry; //
  // while(flightFile.available()){
  //   byte = flightFile.read();
  //   Serial.println(byte);
  //   delay(100);
  // }

  for(int i = 0; i < ITEMS_LOGGED; i++){ 
    entry = "";
    while(simFile.available()){
      byte = simFile.read();
      //if(byte == '\n') continue;
      if(byte == ',' || byte == '\n'){
        simLine[i] = entry;
        break;
      } else
        entry += byte;
    }
  }
}
