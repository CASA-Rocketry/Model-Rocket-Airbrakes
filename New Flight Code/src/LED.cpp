#include <Arduino.h>
#include "Servo.cpp"

#define LED1 2 //green
#define LED2 4 //red
#define BUZZER 5

void initializeLED(){
  pinMode(BUZZER, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  tone(BUZZER, 1000, 1000); //Buzzer test
}

void setTone(int frequency, int duration){
  tone(BUZZER, frequency, duration);
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
    setError(HIGH);
    delay(2000);
    setError(LOW);
    delay(1000);
    for(int i = 0; i < code; i++){
      setError(HIGH);
      delay(300);
      setError(LOW);
      delay(300);
    }
    delay(700);
  }
}

void setError(int level){
  setRedLED(level);
  if(level == 1){
    tone(BUZZER, 300);
  } else
    noTone(BUZZER);
}


void setLEDColor(int r, int g, int b){
  analogWrite(LEDR, 255 - r); //High and low are inverted
  analogWrite(LEDG, 255 - g);
  analogWrite(LEDB, 255 - b);
}