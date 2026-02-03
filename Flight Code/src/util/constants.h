#pragma once

/*
Includes all non-configurable constants that don't relate to hardware addresses
These exist so that the code is easy to read from the names
Do not use a constant here if it is clear from context what it means (ie. delays or number of iterations)
*/
namespace constants{
    namespace airbrake{
        const double MAX_DEPLOYMENT_DEGREES = 170.0;
        const double MAX_VELOCITY_DEPLOYMENT_PER_SECOND = 3.0; //TODO, determine this
    }

    namespace physics{
        const double GRAVITY = 9.81;
        const double ITERATION_TIME_STEP = 0.01;
        const double SEA_LEVEL_PRESSURE = 1013.15; //hPa
    }

    namespace electrical{
        const double BATT_VOLTAGE_SCALER = 3.2788 * 3.3 / 1023; //Voltage divider factor * max voltage / 2^10 - 1
    }
}