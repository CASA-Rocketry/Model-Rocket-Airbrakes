#define LED_PIN 2

void initializeLED(){
  pinMode(LED_PIN, OUTPUT);
}

void setLED(int value){
  digitalWrite(LED_PIN, value);
}

//Shuts down active control and displays error message. code is a positive integer and corresponds to different failure points
void enterErrorMode(int code){
  setServoDeployment(0); //Retract servo
  while(true){
    setLED(HIGH);
    delay(2000);
    setLED(LOW);
    delay(1000);
    for(int i = 0; i < code; i++){
      setLED(HIGH);
      delay(300);
      setLED(LOW);
      delay(300);
    }
    delay(700);
  }
}



