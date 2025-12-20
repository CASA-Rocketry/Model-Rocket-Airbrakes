#include "Airbrake.h"
#include "../hardwareMap.h"
#include "../UI/UI.h"
#include <Arduino.h>
#include "constants.h"

Airbrake::Airbrake(){
    servo.attach(hardwareMap::PWM5);
    setDeployment(0);
}

void Airbrake::test(){
    //Warning beeps
    for(int i = 0; i < 5; i++){
        UI::setTone(500, 500);
        delay(1000);
    }

    //Slow pass
    for(double a = 0; a <= PI; a += PI/500){
        setDeployment(sin(a));
        delay(10);
    }

    //Fast pass
    delay(1000);
    open();
    delay(1000);
    close(); 
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
    servo.write(val * constants::airbrake::MAX_DEPLOYMENT_DEGREES);
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

double Airbrake::getCD(double deployment){
    return deployment * 0.8;
}

//TODO: determine mapping constants
void Airbrake::setCD(double cd){
    setDeployment(cd / 0.8);
}