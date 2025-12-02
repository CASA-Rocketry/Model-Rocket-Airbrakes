#include <Servo.h>

class Airbrake{
private:
    Servo servo;
    float deployment;

public:
    Airbrake();
    void test();
    void setDeployment(double);
    void close();
    void open();
    double getCD();
    void setCD(double);
};