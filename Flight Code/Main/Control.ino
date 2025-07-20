#define TARGET_APOGEE 229
#define TARGET_DURATION 37.5 //36s to 39s 0 points

void runApogeeControl(){

}


//Bang-Bang controller for duration correction
void runTimeControl(){
  if(getTimeEstimate() > TARGET_DURATION)
    setServoDeployment(0);
  else
    setServoDeployment(1);
}