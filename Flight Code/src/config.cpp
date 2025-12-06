//File for holding the constants that are changed in onboard config file
//Include unit for clarity of variable
#include <string>

namespace config{
    namespace kinematics{
        //Rocket Kinematics 
        double BURNOUT_MASS_KG;
        double TARGET_APOGEE_METERS;
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

        //Control parameters
        double KP; //Multiplied in addition to using dt
    };
    
    void fill(std::string config){

    }

};


