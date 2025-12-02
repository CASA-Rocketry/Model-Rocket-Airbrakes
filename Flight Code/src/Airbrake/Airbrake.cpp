#include "Airbrake.h"
#include "../hardwareMap.cpp"

Airbrake::Airbrake(){
    servo.attach(PWM1);
    setDeployment(0);
}

void Airbrake::test(){

}

//Sets servo from 0 to 1 and updates deployment
void Airbrake::setDeployment(double val){
    //Clamp val 
    if(val > 1) 
        val = 1;
    else if (val < 0)
        val = 0;

    deployment = val;
    
    //TODO: add mapping
    servo.write(val * 180);
}

void Airbrake::close(){
    setDeployment(0);
}

void Airbrake::open(){
    setDeployment(1);
}

//TODO: determine mapping constants
double Airbrake::getCD(){
    return deployment * 0.8;
}

//TODO: determine mapping constants
void Airbrake::setCD(double cd){
    setDeployment(cd / 0.8);
}