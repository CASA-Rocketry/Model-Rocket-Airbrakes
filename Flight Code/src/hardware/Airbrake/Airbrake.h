#include <PWMServo.h>

class Airbrake{
private:
    PWMServo servo;
    float deployment;
    bool enabled;
public:
    Airbrake();
    void test();
    void initialize();
    void setDeployment(double);
    void close();
    void open();
    double getCD();
    void setCD(double);
    void enable();
    void disable();
    static double getCD(double);
};