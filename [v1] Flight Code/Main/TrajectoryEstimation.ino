//Uses the current servo deployment to calculate the new drag coefficient
float getDragCoefficient(){
  return 0;//TODO: determine these constants or use a more sophisticated model. 
}

//Returns estimated apogee in meters
float getApogeeEstimate(){
  float apogeeEstimate = 0; //Insert math here
  logLine[6] = String(apogeeEstimate);
  return apogeeEstimate;
}

//Returns estiamted flight time in seconds
float getTimeEstimate(){
  float deltaT = getYEstimate() / getVEstimate(); //Assumes no acceleration
  float currentTime = (timeMillis - launchMillis)/1000.0;
  return currentTime + deltaT;
}