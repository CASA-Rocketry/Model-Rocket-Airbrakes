#include <cmath>

#define BURNOUT_MASS 0.617
#define GRAVITY 9.810245
#define ROCKET_AREA 0.002463
#define AIR_DENSITY 1.18

//Uses the current servo deployment to calculate the new drag coefficient
float getDragCoefficient(){
  return 0.6 + 0;//getServoDeployment() * 0.8;
}

//Returns estimated apogee in meters
float getApogeeEstimate(){
  float apogeeEstimate;
  float k = 0.5 * getDragCoefficient() * ROCKET_AREA * AIR_DENSITY;
  float logArg = (k * getVEstimate() * getVEstimate()) / (BURNOUT_MASS * GRAVITY) + 1;
  if(logArg <= 0)
    apogeeEstimate = getYEstimate();
  else{
    float deltaY = (BURNOUT_MASS/(2*k)) * std::log(logArg);
    apogeeEstimate =  getYEstimate() + deltaY;//Insert math here
  }
  //apogeeEstiamte = getYEstimate()
  logLine[10] = String(apogeeEstimate);

  return apogeeEstimate;
}

// float getApogeeEstimate(){
//   float apogeeEstimate;
//   float k = 0.5 * getDragCoefficient() * ROCKET_AREA * AIR_DENSITY;

//   float deltaY = (0.5 * BURNOUT_MASS * getVEstimate() * getVEstimate()) / (BURNOUT_MASS * GRAVITY + k * getVEstimate() * getVEstimate());
//   apogeeEstimate = getYEstimate() + deltaY;
//   logLine[10] = String(apogeeEstimate);
//   return apogeeEstimate;
// }

//Returns estiamted flight time in seconds
float getTimeEstimate(){
  float deltaT = getYEstimate() / getVEstimate(); //Assumes no acceleration
  float currentTime = (timeMillis - launchMillis)/1000.0;
  return currentTime + deltaT;
}