#include <BasicLinearAlgebra.h>

// //Noise estimates
#define ALT_STD 0.173932 //noise of position observation
// //#define ACC_STD 
#define MODEL_Y_STD 0.01
#define MODEL_V_STD 0.01
#define MODEL_A_STD 0.01

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
  phi = {1, dt, 0.5 * dt * dt,
         0, 1, dt,
         0, 0, 1};

  //Update Kalman Gain
  K = P * ~H * Inverse((H * P * ~H + R));
  //Serial.println(K);
  //Update Estimate
  x = x + K * (z - H * x);
  //Serial.println(x);
  //Update Covariance
  P = (I - K * H) * P;
  //Serial.println(P);
  //Project to next time stamp
  x = phi * x;
  P = phi * P * ~phi + Q;
}



void logStateEstimation(){
  logItem(getYEstimate());
  logItem(getVEstimate());
  logItem(getAEstimate());
  //logItem(getPredictedApogee());

  //Serial output
  Serial.print("estimate:");
  Serial.print(getYEstimate());
  Serial.print(", ");
  Serial.print("measurement:");
  Serial.println(z(0));
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