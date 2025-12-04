//File for holding the constants that are changed in onboard config file
//Include unit for clarity of variable

struct {
    //Rocket Kinematics 
    double BURNOUT_MASS_KG;
    double TARGET_APOGEE_METERS;
    double AIR_DENSITY_KG_PER_METERS_CUBED;

    //Drag constants
    double ROCKET_CD;
    double AIRBRAKE_CD_FULL_DEPLOYMENT;
    double ROCKET_AREA_METERS_SQUARED; //NOT 0.2463

    //Kalman Filter STDs
    double KALMAN_MODEL_STD_Y;
    double KALMAN_MODEL_STD_V;
    double KALMAN_MODEL_STD_A;

    double KALMAN_MEASUREMENT_STD_Y; //altimeter std
    double KALMAN_MEASUREMENT_STD_A; //accelerometer std

    //State transition thresholds
    double LAUNCH_ACCELERATION_METERS_PER_SECOND_SQUARED;
    double COAST_LOCKOUT_SECONDS;

    //Control parameters
    double KP; //Multiplied in addition to using dt

} config;