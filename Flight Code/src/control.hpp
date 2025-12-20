#include "config.h"
#include "constants.h"
#include "Airbrake/Airbrake.h"

//Computes apogee from given altitude, velocity, and cd
//The rest of the constants are pulled from the config
double getApogee(double y, double v, double cd){
    double k = 0.5 * cd * config::ROCKET_AREA_METERS_SQUARED * config::AIR_DENSITY_KG_PER_METERS_CUBED;
    double logArg = (k * v * v) / (config::BURNOUT_MASS_KG * constants::physics::GRAVITY);
    return y + std::log(logArg) * config::BURNOUT_MASS_KG / (2 * k);
}

//Numerically solves for cd to reach target apogee in config
//Computes 
double computeDeployment(double y, double v){
    //Check bounds 
    if(getApogee(y, v, Airbrake::getCD(0)) < config::TARGET_APOGEE_METERS)
        return 0;
    else if(getApogee(y, v, Airbrake::getCD(1)) > config::TARGET_APOGEE_METERS)
        return 1;

    double addition, deployment = 0;
    for(int digits = 1; digits <= 6; digits++){
        addition = 1 >> digits; 
        if(getApogee(y, v, Airbrake::getCD(deployment + addition)) > config::TARGET_APOGEE_METERS)
            deployment += addition;
    }
}