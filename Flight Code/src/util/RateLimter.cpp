#include "RateLimiter.h"

RateLimiter::RateLimiter(double initialVal, double maxVelocitySecond){
    currentVal = initialVal;
    maxVelocityMillis = maxVelocitySecond / 1000;
    lastMillis = millis();
}

double RateLimiter::get(double requestedVal){
    unsigned long currentMillis = millis();
    int deltaMillis = currentMillis - lastMillis;
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