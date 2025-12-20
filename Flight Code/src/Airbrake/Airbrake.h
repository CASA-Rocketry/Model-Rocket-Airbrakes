#include <PWMServo.h>

class Airbrake{
private:
    PWMServo servo;
    float deployment;

public:
    Airbrake();
    void test();
    void setDeployment(double);
    void close();
    void open();
    double getCD();
    void setCD(double);
    static double getCD(double);
};