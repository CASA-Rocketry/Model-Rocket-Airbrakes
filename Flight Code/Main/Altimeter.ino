#include "src/Libraries/Arduino_LPS22HB/Arduino_LPS22HB.h"

//Parameter constnats
#define SEALEVEL_PRESSURE_KPA 101.325

#define CALIBRATION_POINT 0.0
#define CALIBRATION_SAMPLE_SIZE 100
#define CALIBRATION_SAMPLE_RATE 20  //ms

float offset;

void initializeAlt() {
  if (!BARO.begin()) {
    Serial.println("ERROR Initilizing Altimeter");
    enterErrorMode(1);
  }

  BARO.setOutputRate(RATE_75_HZ);
}

double getPressure() {
  double pressure = BARO.readPressure();
  logLine[1] = String(pressure);
  return pressure;
}

double getTemperature() {
  double temperature;
  if(SIMULATION)
    temperature = simLine[RAW_TEMPERATURE_LOG].toFloat();
  else
    temperature = BARO.readTemperature();
  logLine[2] = String(temperature);
  return temperature;
}

float getCalibratedAlt() {
  float calibratedAlt;
  if (SIMULATION)
    calibratedAlt = simLine[6].toFloat();
  else
    calibratedAlt = BARO.readAltitude() + offset;
  logLine[6] = String(calibratedAlt);

  return calibratedAlt;
}

void calibrateAlt() {
  float sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLE_SIZE; i++) {
    sum += BARO.readAltitude();
    //LED blinking
    if (i % 10 == 5)
      setGreenLED(HIGH);
    else if (i % 10 == 0)
      setGreenLED(LOW);

    delay(CALIBRATION_SAMPLE_RATE);
  }
  setGreenLED(LOW);

  float average = sum / CALIBRATION_SAMPLE_SIZE;
  offset = CALIBRATION_POINT - average;

  
  Serial.print("Calibration complete! ... Average: ");
  Serial.println(average);
}
