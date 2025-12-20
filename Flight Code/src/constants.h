//Includes all non-configurable constants that don't relate to hardware addresses
namespace constants{
    namespace airbrake{
        const double MAX_DEPLOYMENT_DEGREES = 170.0;
    }

    namespace physics{
        const double GRAVITY = 9.81;
    }

    namespace electrical{
        const double BATT_VOLTAGE_SCALER = 3.2788 * 3.3 / 1023;
    }
}