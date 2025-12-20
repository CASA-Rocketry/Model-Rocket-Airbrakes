#include <PWMServo.h>

class Canard{
private:
    PWMServo servo;
public:
    Canard();
    Canard(int);
    void setPin(int);
    void setX(double);
};