// #include <Kalman.h>
// using namespace BLA;

// // //Physical constants
// #define g 9.8

// //Matrix dimensions
// #define NUM_STATE 3 //(y, v, a)
// #define NUM_OBS 1 //(y)

// //Noise estimates
// #define ALT_STD 0.278001153 //noise of position observation
// //#define ACC_STD 
// #define MODEL_Y_STD 0.1
// #define MODEL_V_STD 0.1
// #define MODEL_A_STD 0.1

// BLA::Matrix<NUM_OBS> obs; //observation vector
// KALMAN<NUM_STATE, NUM_OBS> K; //Kalman filter

// void initializeKalmanFilter(){
//   K.F = {1.0, 0, 0,
//         0, 1.0, 0,
//         0, 0, 1.0}; //State evolution (updated with DT)
//   K.H = {1, 0, 0}; //State to measurement
//   K.R = {ALT_STD * ALT_STD}; //Measurement covariance
//   K.Q = {MODEL_Y_STD * MODEL_Y_STD, 0, 0,
//           0, MODEL_V_STD * MODEL_V_STD, 0,
//           0, 0, MODEL_A_STD * MODEL_A_STD}; //Model covariance matrix
// }

// void updateKalmanFilter(float dT){
//   float altitudeReading = getCalibratedAlt();
  
//   obs.Fill(altitudeReading); //Create measurement matrix
//   //Update state transition matrix
//   K.F = {1.0,  dt,  dt*dt/2,
// 		    0.0,  1.0,   dt,
//          0.0, 0.0,  1.0};

//   K.update(obs);

//   //Print output for tuning
//   Serial.print("alt:");
//   Serial.print(altitudeReading);
//   Serial.print(", ");
  
//   Serial.print("Kx0:");
//   Serial.print(K.x(0));
//   Serial.print(", ");
//   Serial.print("Kx1:");
//   Serial.print(K.x(1));
//   Serial.print(", ");
//   Serial.print("Kx2:");
//   Serial.println(K.x(2));


// }



// void logControl(){
//   logItem(K.x(0));
//   logItem(K.x(1));
//   logItem(K.x(2));
//   logItem(getPredictedApogee());
// }

float getYEstimate(){
//   return K.x(0);
return 0;
}

float getVEstimate(){
//    return K.x(1);
  return 0;
}

float getAEstimate(){
//   return K.x(2);
  return 0;
}