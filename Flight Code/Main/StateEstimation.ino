#include <BasicLinearAlgebra.h>

// //Noise estimates
#define ALT_STD 0.173932 //noise of position observation
// //#define ACC_STD 
#define MODEL_Y_STD 0.02
#define MODEL_V_STD 0.5
#define MODEL_A_STD 0.1

BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1}; //3x3 identity matrix used for later
BLA::Matrix<3> x = {0, 0, 0}; //state estimate
BLA::Matrix<1> z = {0}; //measurement
BLA::Matrix<3, 3> phi = I; //state transition
BLA::Matrix<1, 3> H = {1, 0, 0}; //state to measurement
BLA::Matrix<1> R = {ALT_STD * ALT_STD}; //measurement covariance
BLA::Matrix<3, 3> Q = {MODEL_Y_STD * MODEL_Y_STD, 0, 0,
                                      0, MODEL_V_STD * MODEL_V_STD, 0,
                                      0, 0, MODEL_A_STD * MODEL_A_STD}; //Process covariance
BLA::Matrix <3, 3> P = I; //Error covariance
BLA::Matrix <3> K; //Kalman gain


void updateKalmanFilter(){
  //Add data
  z(0) = getCalibratedAlt();
  //endProcess("Altitude calculation");

  //beginProcess();
  phi = {1, dt, 0.5 * dt * dt,
         0, 1, dt,
         0, 0, 1};
  //endProcess("Kalman - phi creation");

  //Update Kalman Gain
  //beginProcess();
  K = P * ~H * Inverse((H * P * ~H + R));
  //endProcess("Kalman - update gain");

  //Update Estimate
  //beginProcess();
  x = x + K * (z - H * x);
  //endProcess("Kalman - update estimate");

  //Update Covariance
  //beginProcess();
  P = (I - K * H) * P;
  //endProcess("Kalman - update covariance");

  //Project to next time stamp
  //beginProcess();
  x = phi * x;
  P = phi * P * ~phi + Q;
  //endProcess("Kalman - project");

  //Update log
  //beginProcess();
  logLine[7] = String(x(0));
  logLine[8] = String(x(1));
  logLine[9] = String(x(2));
  //endProcess("Kalman - log");
}

float getYEstimate(){
  return x(0);
}

float getVEstimate(){
  return x(1);
}

float getAEstimate(){
  return x(2);
}