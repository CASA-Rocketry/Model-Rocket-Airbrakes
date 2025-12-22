#include <PWMServo.h>

class Airbrake{
private:
    PWMServo servo;
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
    float deployment;
};