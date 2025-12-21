//File for holding the constants that are changed in onboard config file
//Include unit for clarity of variable
#pragma once

#include <string>
#include "print.h"

class Config{
private:
    bool getBool(std::string);
    std::string cleanString(std::string);
    void fillConfig(std::string[]);
    void parseConfig(std::string, std::string[]);
public:
    const int CONFIG_VALUES = 19;
    std::string configString; //stores raw string before configuring

    //Rocket Kinematics 
    double BURNOUT_MASS_KG;
    double AIR_DENSITY_KG_PER_METERS_CUBED;

    //Drag constants
    double ROCKET_CD;
    double AIRBRAKE_CD_FULL_DEPLOYMENT; //Only component from airbrake
    double ROCKET_AREA_METERS_SQUARED; //NOT 0.2463


    //Kalman filter
    double MODEL_STD_Y;
    double MODEL_STD_V;
    double MODEL_STD_A;

    double MEASUREMENT_STD_Y; //altimeter std
    double MEASUREMENT_STD_A; //accelerometer std


    //Mode
    bool SIMULATION;
    bool AIRBRAKES_ENABLED;

    //Log
    std::string SIM_NAME;
    std::string LOG_NAME;

    //State transition thresholds
    double LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED;
    double COAST_LOCKOUT_SECONDS;
    int ALTIMETER_LOCKOUT_SECONDS;
    double TARGET_APOGEE_METERS;
    double KP; //Multiplied in addition to using dt
    void printCheck();
    void configureConstants(std::string);
};

