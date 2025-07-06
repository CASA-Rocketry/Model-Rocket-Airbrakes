#include <Servo.h>
#include <SPI.h>
#include <SD.h>


enum FlightMode {
  LAUNCH_PAD = 0,
  THRUST = 1,
  COAST = 2,
  RECOVERY = 3,
  LAND = 4
}

enum FlightMode mode = LAUNCH_PAD;

//Hardware
Servo servo;


double t_i, t_f = 0;
double y_i, y_f = 0;
double v_i, v_f = 0;

double servoCommand = 0;


void setup() {
  //Initialize hardware
  servo.attach(0);
}

void loop() {
  sample();

  //Mange flight states
  switch(mode){
    case LAUNCH_PAD:
      if (v_f > 1)
        mode++;
    case THRUST:
      //Add accelerometer data
    case COAST:
      //Add airbrake control for apogge
      if (v_f < 0)
        mode++;
    case RECOVER:
      //Add airbrake control for flight time
      if(v_f > -0.5 && v_f < 0.5)
        mode++;
    case LAND:
      servoCommand = 0;
  }

  updateServo();
  log();
}

void updateServo(){
  servo.write(servoCommand);
}

void log(){
  //log raw sensor inputs, filtered/calculated values, flight mode, and servo output to sd card
}

void sample(){
  updateAltitude();
  updateVerticalVelocity();
}

//Returns the altitude in meters
void updatAltitude(){
  //TODO: fuse barometer and accelerometer data using kalman filter

  //Shift back samples
  y_i = y_f;
  t_i = t_f;

  //Update new samples
  y_f = 0; //Use sensor here
  t_f = millis() / 1000.;
}

//Returns vy in m/s
void updateVerticalVelocity(){
  v_i = v_f;
  v_f = (y_f - y_i) / (t_i - t_f);
}