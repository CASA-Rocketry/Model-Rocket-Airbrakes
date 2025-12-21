#include "Config.hpp"

//Stores values to config from configValues
void Config::fillConfig(std::string configValues[]){     
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
void Config::parseConfig(std::string config, std::string configValues[]){
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

        if(constantsFilled != CONFIG_VALUES)
            sPrintln("Incorrent number of config constants found");
}


void Config::configureConstants(std::string config){
    configString = config;
    std::string configValues[CONFIG_VALUES] = {};
    parseConfig(config, configValues);
    fillConfig(configValues);
}

//Converts F/T to false/true
bool Config::getBool(std::string str){
    if(str.compare("F") == 0)
        return false;
    return true;
}

//Removes unwanted (\n and \r) chartacters from string that may be left over
std::string Config::cleanString(std::string str){
    while(str.find_first_of("\r\n") != std::string::npos){
        str.erase(str.find_first_of("\r\n"), 1);
    }
    return str;
}