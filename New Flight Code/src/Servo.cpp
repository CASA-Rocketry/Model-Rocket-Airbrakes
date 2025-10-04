#include <Servo.h>
#include <Arduino.h>

#define PWM1 3
#define PWM2 5
#define PWM3 6

//Constnats
#define MIN_DEPLOYMENT_DEGREES 0
#define MAX_DEPLOYMENT_DEGREES 180

Servo servo;


void initializeServo(){
  servo.attach(PWM1);

  //Slow pass
  for(double a = 0; a <= PI; a += PI/100){
    setServoDeployment(sin(a));
    delay(50);
  }

  //Fast pass
  delay(1000);
  setServoDeployment(1);
  delay(1000);
  setServoDeployment(0);
  delay(1000); 
}


//Deployment ranges from 0 to 1
void setServoDeployment(float deployment){
  logLine[11] = String(deployment);
  double clampedDeployment = clamp(deployment, 0, 1); //May remove if this is verified earlier
  double angle = MIN_DEPLOYMENT_DEGREES + (MAX_DEPLOYMENT_DEGREES - MIN_DEPLOYMENT_DEGREES) * clampedDeployment;
  
  servo.write(angle);
}


//Utility math function
double clamp(double value, double min, double max){
  if(value > max)
    return max;
  if(value < min)
    return min;
  return value;
}