#pragma once

#include <Arduino.h>

//Class for managing rate limiting
//Timing is handled internally by calling millis()
class RateLimiter {
private:
    double currentVal;
    unsigned long lastMillis; 
    double maxVelocityMillis; //delta val / delta t in ms
public:
    RateLimiter(double = 0, double = 0);
    void setMaxVelocity(double);
    double get(double);
    void updateCurrent(double);
};