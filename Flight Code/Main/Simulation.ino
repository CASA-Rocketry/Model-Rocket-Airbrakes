#include <String>

void initializeSim(){
  getNewLine(); //Clear header titles 
  getNewLine(); //Load first line
}

void updateSim(){
  getNewLine();
}

void getNewLine () {
  if (!flightFile.available())
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
    while(flightFile.available()){
      byte = flightFile.read();
      //if(byte == '\n') continue;
      if(byte == ',' || byte == '\n'){
        simLine[i] = entry;
        //Serial.println(entry);
        break;
      } else
        entry += byte;
    }
  }
}
