#define LED1 2 //green
#define LED2 4 //red

void initializeLED(){
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void setRedLED(int value){
  digitalWrite(LED2, value);
}

void setGreenLED(int value){
  digitalWrite(LED1, value);
}

//Shuts down active control and displays error message. code is a positive integer and corresponds to different failure points
void enterErrorMode(int code){
  setServoDeployment(0); //Retract servo
  while(true){
    setRedLED(HIGH);
    delay(2000);
    setRedLED(LOW);
    delay(1000);
    for(int i = 0; i < code; i++){
      setRedLED(HIGH);
      delay(300);
      setRedLED(LOW);
      delay(300);
    }
    delay(700);
  }
}


void setLEDColor(int r, int g, int b){
  analogWrite(LEDR, 255 - r); //High and low are inverted
  analogWrite(LEDG, 255 - g);
  analogWrite(LEDB, 255 - b);
}