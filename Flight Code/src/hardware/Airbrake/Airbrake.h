#include <PWMServo.h>
#include "../../util/RateLimiter.h"
#include "../../util/constants.h"

class Airbrake{
private:
    PWMServo servo;
    RateLimiter currentPositionRateLimiter{0, constants::airbrake::MAX_VELOCITY_DEPLOYMENT_PER_SECOND};
    bool enabled;
public:
    Airbrake();
    void test();
    void initialize();
    void setDeployment(double);
    void close();
    void open();
    void enable();
    void disable();
    float commandedDeployment, currentDeployment;
};