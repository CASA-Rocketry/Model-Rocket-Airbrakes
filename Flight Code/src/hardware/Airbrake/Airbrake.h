#include <PWMServo.h>

class Airbrake{
private:
    PWMServo servo;
    float deployment;

public:
    void test();
    void initialize();
    void setDeployment(double);
    void close();
    void open();
    double getCD();
    void setCD(double);
    static double getCD(double);
};