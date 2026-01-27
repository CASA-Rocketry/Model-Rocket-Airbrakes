#include "RateLimiter.h"

RateLimiter::RateLimiter(double initialVal, double maxVelocitySecond){
    currentVal = initialVal;
    maxVelocityMillis = maxVelocitySecond / 1000;
    lastMillis = millis();
}

//Only returns current time stamp, airbrake will stop moving if not commanded again
double RateLimiter::get(double requestedVal){
    //Update time stamps
    unsigned long currentMillis = millis();
    int deltaMillis = currentMillis - lastMillis;
    lastMillis = currentMillis; //update for NEXT time stamp

    double deltaVal = requestedVal - currentVal;
    double maxDeltaVal = deltaMillis * maxVelocityMillis;

    //Clamp deltaVal between +- maxDeltaVal
    if(maxDeltaVal < deltaVal)
        deltaVal = maxDeltaVal;
    else if(deltaVal < -maxDeltaVal)
        deltaVal = -maxDeltaVal;

    currentVal += deltaVal;
    return currentVal;
}

void RateLimiter::updateCurrent(double newCurrentVal){
    currentVal = newCurrentVal;
}