#include "CanardSet.h"

CanardSet::CanardSet(){
    //Attach canards to pins
    c1.setPin(hardwareMap::PWM1);
    c2.setPin(hardwareMap::PWM2);
    c3.setPin(hardwareMap::PWM3);
    c4.setPin(hardwareMap::PWM4);
}

//tx = pitch forward/backward
//ty = pitch left/right
//tz = roll
void CanardSet::setTorque(double tx, double ty, double tz){

}