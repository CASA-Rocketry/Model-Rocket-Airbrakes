#include "Trigger.h"
#include <Arduino.h>


//Returns true if the trigger is true for at least millis
bool Trigger::getHoldState(bool val, unsigned long thresholdMillis){
    if(val){
        if(!previousState){ //first press
            timeOfPressMillis = millis();
            previousState = true;
        } else if(millis() - timeOfPressMillis >= thresholdMillis) //only check duration if not first press
                return true;
    }
    previousState = false;
    return false;
}

//Properly resets values so trigger can be used multiple times
void Trigger::reset(){
    previousState = false;
}