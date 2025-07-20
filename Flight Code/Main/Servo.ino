#include <Servo.h>

#define PWM1 3
#define PWM2 5
#define PWM3 6

//Constnats
#define MIN_DEPLOYMENT_DEGREES 0
#define MAX_DEPLOYMENT_DEGREES 180

Servo servo;
float servoDeployment = 0;

void initializeServo(){
  servo.attach(PWM1);
  setServoDeployment(0);
}


//Deployment ranges from 0 to 1
void setServoDeployment(double deployment){
  double clampedDeployment = clamp(deployment, 0, 1); //May remove if this is verified earlier
  servoDeployment = clampedDeployment;
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