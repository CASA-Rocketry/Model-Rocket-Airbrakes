#include "Canard.h"
#include "../hardwareMap.cpp"


class CanardSet{
private:    
    Canard c1, c2, c3, c4;
public:
    CanardSet();
    void setTorque(double, double, double);
};