#include "StateEstimator.h"

StateEstimator::StateEstimator(){
    x = {};
    phi = {};
    h = {1, 0, 0,
        0, 0, 1};
    r = {};
    p = {};
    z = {};
    i = {1, 0, 0,
        0, 1, 0,
        0, 0, 1};
}

void StateEstimator::setMeasurementSTD(double ySTD, double aSTD){
    r = {ySTD * ySTD, 0,
        0, aSTD * aSTD};
}

void StateEstimator::fillFromConfig(Config& config){
    q = {config.MODEL_STD_Y, 0, 0,
            0, config.MODEL_STD_V, 0,
            0, 0, config.MODEL_STD_A};
    setMeasurementSTD(config.MEASUREMENT_STD_Y, config.MEASUREMENT_STD_A); //initializes R with default config values
}

void StateEstimator::update(double yMeasurement, double aMeasurement, double dtSeconds){
    //Update measuremennt and state-transition
    phi = {1, dtSeconds, 0.5 * dtSeconds * dtSeconds,
            0, 1, dtSeconds,
            0, 0, 1};
    z = {yMeasurement, aMeasurement};

    //Update kalman gain
    k = p * (~h) * BLA::Inverse(h * p * (~h) + r);

    //Update state from measurment
    x = x + k * (z - h * x);

    //Update covariance
    p = (i - k * h) * p;

    //Project to next time step
    x = phi * x;
    p = phi * p * (~phi) + q;

}

double& StateEstimator::y(){
    return x(0);
}

double& StateEstimator::v(){
    return x(1);
}

double& StateEstimator::a(){
    return x(2);
}

