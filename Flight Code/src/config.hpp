//File for holding the constants that are changed in onboard config file
//Include unit for clarity of variable
#pragma once

#include <string>
#include "Log/print.h"

namespace config{
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
    double ALTIMETER_LOCKOUT_SECONDS;
    double TARGET_APOGEE_METERS;
    double KP; //Multiplied in addition to using dt

    bool getBool(std::string);
    std::string cleanString(std::string);

    //Stores values to config from configValues
    void fillConfig(std::string configValues[]){     
        SIMULATION = getBool(configValues[0]);
        AIRBRAKES_ENABLED = getBool(configValues[1]);

        LOG_NAME = cleanString(configValues[2]);
        SIM_NAME = cleanString(configValues[3]);

        BURNOUT_MASS_KG = std::stod(configValues[4]);
        AIR_DENSITY_KG_PER_METERS_CUBED = std::stod(configValues[5]);
        ROCKET_CD = std::stod(configValues[6]);
        AIRBRAKE_CD_FULL_DEPLOYMENT = std::stod(configValues[7]);
        ROCKET_AREA_METERS_SQUARED = std::stod(configValues[8]);
        
        MODEL_STD_Y = std::stod(configValues[9]);
        MODEL_STD_V = std::stod(configValues[10]);
        MODEL_STD_A = std::stod(configValues[11]);
        MEASUREMENT_STD_Y = std::stod(configValues[12]);
        MEASUREMENT_STD_A = std::stod(configValues[13]);

        TARGET_APOGEE_METERS = std::stod(configValues[14]);
        LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED = std::stod(configValues[15]);
        COAST_LOCKOUT_SECONDS = std::stod(configValues[16]);
        KP = std::stod(configValues[17]);

        ALTIMETER_LOCKOUT_SECONDS = std::stod(configValues[18]);
    }

    //Accepts config in string form and adds values to configValues
    //TODO: fix this bad indexing
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
                sPrintln(entry.c_str());
                constantsFilled++;
            }

            if(constantsFilled != CONFIG_VALUES)
                sPrintln("Incorrent number of config constants found");
    }


    void configureConstants(std::string config){
        configString = config;
        std::string configValues[CONFIG_VALUES] = {};
        parseConfig(config, configValues);
        fillConfig(configValues);
        sPrintln(KP);
    }

    //Converts F/T to false/true
    bool getBool(std::string str){
        if(str.compare("F") == 0)
            return false;
        return true;
    }

    //Removes unwanted (\n and \r) chartacters from string that may be left over
    std::string cleanString(std::string str){
        while(str.find_first_of("\r\n") != std::string::npos){
            str.erase(str.find_first_of("\r\n"), 1);
        }
        return str;
    }
};



