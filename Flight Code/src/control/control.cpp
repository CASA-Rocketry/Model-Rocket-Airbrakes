#include "../util/Config.hpp"
#include "../util/constants.h"
#include "../hardware/Airbrake/Airbrake.h"
#include "control.h"

namespace control{
    RateLimiter rateLimiter(0, 0);
}

//Computes apogee from given altitude, velocity, and cd
//The rest of the constants are pulled from the config
double control::getApogee(double y, double v, double deployment, Config& config){
    double cd = control::getCD(deployment, config);
    double k = 0.5 * cd * config.ROCKET_AREA_METERS_SQUARED * config.AIR_DENSITY_KG_PER_METERS_CUBED;
    double logArg = (k * v * v) / (config.BURNOUT_MASS_KG * constants::physics::GRAVITY) + 1;
    return y + std::log(logArg) * config.BURNOUT_MASS_KG / (2 * k);
}

double control::getApogeeIterative(double y, double v, double deployment, Config& config){
    double cd = control::getCD(deployment, config);
    double kOverMass = 0.5 * cd * config.AIR_DENSITY_KG_PER_METERS_CUBED * config.ROCKET_AREA_METERS_SQUARED / config.BURNOUT_MASS_KG;
    double a;
    //Changes y and v, so they must be passed by value
    while(v > 0){ //Repeat until apogee found 
        a = -constants::physics::GRAVITY - kOverMass * v * v;
        v += a * constants::physics::ITERATION_TIME_STEP; //Could change step size inversely proportional to velocity for better precision
        y += v * constants::physics::ITERATION_TIME_STEP;
    }
    return y;
}


//Should be called immediately before starting rate limiting 
void control::startRateLimiter(Config& config){
    rateLimiter = RateLimiter(0, config.CONTROL_MAX_DEPLOYMENT_PER_SECOND);
}

//Deployment is 0 when it switches to tracking mode 

//Numerically solves for cd to reach target apogee in config
//Computes 
double control::computeDeployment(double y, double v, Config& config){
    //Check bounds 
    if(getApogee(y, v, 0, config) < config.TARGET_APOGEE_METERS)
        return 0;
    else if(getApogee(y, v, 1, config) > config.TARGET_APOGEE_METERS)
        return 1;

    double addition, deployment = 0;
    for(int digits = 1; digits <= 6; digits++){
        addition = 1.0 / (1 << digits); 
        if(getApogee(y, v, deployment + addition, config) > config.TARGET_APOGEE_METERS)
            deployment += addition;
    }
    //Apply rate limit
    return rateLimiter.get(deployment);
}

double control::getCD(double deployment, Config& config){
    return config.ROCKET_CD + deployment * config.AIRBRAKE_CD_FULL_DEPLOYMENT;
}