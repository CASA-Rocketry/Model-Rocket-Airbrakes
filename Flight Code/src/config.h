//File for holding the constants that are changed in onboard config file
//Include unit for clarity of variable
#pragma once

#include <string>
#include "globalSettings.h"



namespace config{
    namespace kinematics{
        //Rocket Kinematics 
        double BURNOUT_MASS_KG;
        
        double AIR_DENSITY_KG_PER_METERS_CUBED;

        //Drag constants
        double ROCKET_CD;
        double AIRBRAKE_CD_FULL_DEPLOYMENT; //Only component from airbrake
        double ROCKET_AREA_METERS_SQUARED; //NOT 0.2463
    };

    namespace kalman{
        double MODEL_STD_Y;
        double MODEL_STD_V;
        double MODEL_STD_A;

        double MEASUREMENT_STD_Y; //altimeter std
        double MEASUREMENT_STD_A; //accelerometer std
    };

    namespace mode{
        bool SIMULATION;
        bool AIRBRAKES_ENABLED;
    };

    namespace log{
        std::string SIM_NAME;
        std::string LOG_NAME;
    };
    
    namespace control{
        //State transition thresholds
        double LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED;
        double COAST_LOCKOUT_SECONDS;

        double ALTIMETER_LOCKOUT_SECONDS;

        //Target parameters
        double TARGET_APOGEE_METERS;

        //Control parameters
        double KP; //Multiplied in addition to using dt
    };
    
    const int CONFIG_VALUES = 19;
};

//Converts F/T to false/true
bool getBool(std::string str){
    if(str.compare("F") == 0)
        return false;
    return true;
}

//Stores values to config from configValues
void fillConfig(std::string configValues[]){     
    config::mode::SIMULATION = getBool(configValues[0]);
    config::mode::SIMULATION = getBool(configValues[1]);

    config::log::LOG_NAME = configValues[2];
    config::log::SIM_NAME = configValues[3];

    config::kinematics::BURNOUT_MASS_KG = std::stod(configValues[4]);
    config::kinematics::AIR_DENSITY_KG_PER_METERS_CUBED = std::stod(configValues[5]);
    config::kinematics::ROCKET_CD = std::stod(configValues[6]);
    config::kinematics::AIRBRAKE_CD_FULL_DEPLOYMENT = std::stod(configValues[7]);
    config::kinematics::ROCKET_AREA_METERS_SQUARED = std::stod(configValues[8]);
    
    config::kalman::MODEL_STD_Y = std::stod(configValues[9]);
    config::kalman::MODEL_STD_V = std::stod(configValues[10]);
    config::kalman::MODEL_STD_A = std::stod(configValues[11]);
    config::kalman::MEASUREMENT_STD_Y = std::stod(configValues[12]);
    config::kalman::MEASUREMENT_STD_A = std::stod(configValues[13]);

    config::control::TARGET_APOGEE_METERS = std::stod(configValues[14]);
    config::control::LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED = std::stod(configValues[15]);
    config::control::COAST_LOCKOUT_SECONDS = std::stod(configValues[16]);
    config::control::KP = std::stod(configValues[17]);
    config::control::ALTIMETER_LOCKOUT_SECONDS = std::stod(configValues[18]);
}

//Accepts config in string form and adds values to configValues
void parseConfig(std::string config, std::string configValues[]){
    int startIndex, breakIndex = -1;
        int constantsFilled = 0;
        
        std::string entry;
        while((startIndex = config.find_first_of(',', breakIndex + 1)) != std::string::npos){
            if(config.find_first_of('\n') == std::string::npos) //Last line
                entry = config.substr(startIndex);
            else {
                breakIndex = config.find_first_of('\n', breakIndex + 1);
                entry = config.substr(startIndex + 1, breakIndex - startIndex - 1);
            }
            configValues[constantsFilled] = entry;
            constantsFilled++;
        }

        if(constantsFilled != config::CONFIG_VALUES)
            sPrintln("Incorrent number of config constants found");
}


void configureConstants(std::string config){
    std::string configValues[config::CONFIG_VALUES] = {};
    parseConfig(config, configValues);
    fillConfig(configValues);
    sPrintln(config::control::KP);
}