#include <Kalman.h>

//Physical constants
#define g 9.8

//Matrix dimensions
#define NUM_STATE 3 //(y, v, a)
#define NUM_OBS 1 //(y)

//Noise estimates
#define SENSOR_STD 0.278001153 //noise of position observation
#define MODEL_Y_STD 0.01
#define MODEL_V_STD 0.01
#define MODEL_A_STD 0.01

BLA::Matrix<NUM_OBS> obs; //observation vector
KALMAN<NUM_STATE, NUM_OBS> K; //Kalman filter

void initializeKalmanFilter(){
  K.F = {1.0, 0, 0,
        0, 1.0, 0,
        0, 0, 1.0}; //State evolution (updated with DT)
  K.H = {1, 0, 0}; //State to measurement
  K.R = {SENSOR_STD * SENSOR_STD}; //Measurement covariance
  K.Q = {MODEL_Y_STD * MODEL_Y_STD, 0, 0,
          0, MODEL_V_STD * MODEL_V_STD, 0,
          0, 0, MODEL_A_STD * MODEL_A_STD}; //Model covariance matrix
}

void updateKalmanFilter(float dT){
  float altitudeReading = getCalibratedAlt();
  
  obs.Fill(altitudeReading); //Create measurement matrix
  //Update state transition matrix
  K.F = {1.0,  dt,  dt*dt/2,
		    0.0,  1.0,   dt,
         0.0, 0.0,  1.0};

  //Print output for tuning
  // Serial.print("alt:");
  // Serial.print(altitudeReading);
  // Serial.print(", ");
  // K.update(obs);
  // Serial.print("Kx0:");
  // Serial.println(K.x(0));

}


float getPredictedApogee(){
  float dy = 0.5 * K.x(1) * K.x(1) / g;
  return K.x(0) + dy;
}

void logControl(){
  logItem(K.x(0));
  logItem(K.x(1));
  logItem(K.x(2));
  logItem(getPredictedApogee());
}