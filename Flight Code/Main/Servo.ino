#include <Servo.h>

#define SERVO_PIN 3

//Constnats
#define MIN_DEPLOYMENT_DEGREES 0
#define MAX_DEPLOYMENT_DEGREES 180

Servo servo;

void initializeServo(){
  servo.attach(SERVO_PIN);
  setServoDeployment(0);
}


//Deployment ranges from 0 to 1
void setServoDeployment(double deployment){
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