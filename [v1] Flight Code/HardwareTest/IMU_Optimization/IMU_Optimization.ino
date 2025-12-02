#include "src/Libraries/Arduino_BMI270_BMM150/Arduino_BMI270_BMM150.h"
#include "src/Libraries/Arduino_LPS22HB/Arduino_LPS22HB.h"

float ax, ay, az;
unsigned long processMicros = 0;
int loopIterations = 0;

void setup() {
  // put your setup code here, to run once:
  IMU.begin();
  BARO.begin();
  BARO.setOutputRate(5);
  Serial.begin(9600);
  //IMU.setContinuousMode();
  //IMU.onInterrupt(interrupt);
  //IMU.setInterruptPin()
}

void interrupt(){
  // if(loopIterations++ % 1000 == 0){
  //   endProcess("1000x acceleration read (us)");
  //   beginProcess();
  // }
}


void loop() {
  // if(loopIterations++ % 1000 == 0){
  //   endProcess("1000x acceleration read (us)");
  //   beginProcess();
  // }

  beginProcess();
  //Serial.println(
    BARO.readPressure();
    //);
  endProcess("Pressure grab");
  //Serial.println(az);

  delay(10);

}

//Resets timer for new process timing
void beginProcess(){
  processMicros = micros();
}

//Prints out process time
void endProcess(String processName){
  unsigned long durationMicro = micros() - processMicros;
  Serial.println(processName + ": " + durationMicro + " us");
}
