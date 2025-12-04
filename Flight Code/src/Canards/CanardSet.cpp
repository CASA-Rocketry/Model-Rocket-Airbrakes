#include "CanardSet.h"

CanardSet::CanardSet(){
    //Attach canards to pins
    c1.setPin(PWM1);
    c2.setPin(PWM2);
    c3.setPin(PWM3);
    c4.setPin(PWM4);
}

//tx = pitch forward/backward
//ty = pitch left/right
//tz = roll
void CanardSet::setTorque(double tx, double ty, double tz){

}