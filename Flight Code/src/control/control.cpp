#include "../util/Config.hpp"
#include "../util/constants.h"
#include "../hardware/Airbrake/Airbrake.h"
#include "control.h"

//Computes apogee from given altitude, velocity, and cd
//The rest of the constants are pulled from the config
double control::getApogee(double y, double v, double deployment, Config& config){
    double cd = control::getCD(deployment, config);
    double k = 0.5 * cd * config.ROCKET_AREA_METERS_SQUARED * config.AIR_DENSITY_KG_PER_METERS_CUBED;
    double logArg = (k * v * v) / (config.BURNOUT_MASS_KG * constants::physics::GRAVITY) + 1;
    return y + std::log(logArg) * config.BURNOUT_MASS_KG / (2 * k);
}

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
        addition = 1 >> digits; 
        if(getApogee(y, v, deployment + addition, config) > config.TARGET_APOGEE_METERS)
            deployment += addition;
    }
}

double control::getCD(double deployment, Config& config){
    return config.ROCKET_CD + deployment * config.AIRBRAKE_CD_FULL_DEPLOYMENT;
}