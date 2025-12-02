#include <Servo.h>

#define PWM1 3
#define PWM2 5
#define PWM3 6

//Constnats
#define MIN_DEPLOYMENT_DEGREES 0
#define MAX_DEPLOYMENT_DEGREES 180


Servo servo;
double currentDeployment = 0;


void initializeServo(){
  servo.attach(PWM1);
  sPrintln("Servo initialized");
}

void testServo(){
  sPrintln("Testing servo");
  //Warning sequence for servo test
    for(int i = 0; i < 5; i++){
      setTone(5000, 500);
      delay(1000);
    }
    delay(2000);

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
  servo.write(0);
}


//Deployment ranges from 0 to 1
void setServoDeployment(float deployment){
  
  
  double clampedDeployment = clamp(deployment, 0, 1); //May remove if this is verified earlier
  currentDeployment = clampedDeployment;
  logLine[11] = String(currentDeployment);
  
  if(!SERVO) return;
  double angle = MIN_DEPLOYMENT_DEGREES + (MAX_DEPLOYMENT_DEGREES - MIN_DEPLOYMENT_DEGREES) * clampedDeployment;
  
  servo.write(0);//angle);
}

float getServoDeployment(){
  return currentDeployment;
}


//Utility math function
double clamp(double value, double min, double max){
  if(value > max)
    return max;
  if(value < min)
    return min;
  return value;
}