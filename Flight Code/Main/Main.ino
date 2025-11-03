#define SERIAL true //false during flight
#define SIMULATION false //false on real flight
#define BUZZER true
#define SERVO true

//Shared pin definitions
#define SCK 13
#define MISO 12
#define MOSI 11

#define ITEMS_LOGGED 13
#define TIME_STAMP_LOG 0
#define RAW_PRESSURE_LOG 1
#define RAW_TEMPERATURE_LOG 2
#define RAW_AX_LOG 3
#define RAW_AY_LOG 4
#define RAW_AZ_LOG 5
#define CALCULATED_ALT_LOG 6
#define Y_ESTIMATE_LOG 7
#define V_ESTIMATE_LOG 8
#define A_ESTIMATE_LOG 9
#define APOGEE_ESTIMATE_LOG 10
#define SERVO_DEPLOYMENT_LOG 11
#define FLIGHT_MODE_LOG 12

String logLine[ITEMS_LOGGED] = {};
String simLine[ITEMS_LOGGED] = {};

#define BURN_TIME 1.5
#define ALTIMETER_LOCKOUT 3 //altimeter delay (seconds)

unsigned long timeMillis = 0, timeNewMillis;
unsigned long launchMillis, landMillis;
unsigned long processMicros;
float dt;

enum FlightMode {
  LAUNCH_PAD = 0,
  THRUST = 1,
  COAST = 2,
  RECOVERY = 3,
  LANDING = 4,
  LANDED = 5
} mode = LAUNCH_PAD;

void setup() {
  //Initialize hardware
  initializeLog(); //If sim, this opens the sim file
  initializeLED();

  if (SIMULATION){
    setLEDColor(255, 0, 0); //Red for sim
    initializeSim();
  }
  else {
    setLEDColor(0, 255, 0); //Green for flight
    
    //Warning sequence for servo test
    // for(int i = 0; i < 5; i++){
    //   setTone(5000, 500);
    //   delay(1000);
    // }
    // delay(2000);
  }

  

  if(!SIMULATION){
    initializeServo();
    initializeIMU();

    //setTone(1000, 1000);
    //delay(ALTIMETER_LOCKOUT * 1000);
    initializeAlt();
    setTone(1000, 3000);
  }


  setGreenLED(HIGH); //Signify successful
}

void loop() {
  beginProcess();

  if(SIMULATION)
    updateSim();

  updateTime();
  updateKalmanFilter();

  //beginProcess();
  updateIMU();
  //endProcess("IMU data grab");

  //beginProcess();
  getTemperature();
  //endProcess("Tempterature data grab");
  
  //runApogeeControl();
  //Mange flight states
  switch(mode){
    case LAUNCH_PAD:
      //setLEDColor(255, 0, 0); //RED
      //Serial.println(getVerticalAcceleration());
      if (getVerticalAcceleration() > 20){
        mode = THRUST;
        launchMillis = timeMillis;
      }
      else
        break;
    case THRUST:
      //setLEDColor(240, 122, 5); //Orange
      //Serial.println((timeMillis - launchMillis)/1000.0);
      if((timeMillis - launchMillis)/1000.0 > BURN_TIME) //Care that these are UNSIGNED longs
        mode = COAST;
      else
        break;
    case COAST:
      //setLEDColor(255, 247, 0); //Yellow
      runApogeeControl();
      if (getVEstimate() < 0)
        mode = RECOVERY;
      else
        break;
    case RECOVERY:
      //setLEDColor(0, 255, 0); //Green
      runTimeControl();
      if(getYEstimate() < 5)
        mode = LANDING;
      else 
        break;
    case LANDING:
      //setLEDColor(0, 0, 255); //Blue
      setServoDeployment(0);
      if(abs(getVEstimate()) < 0.2)
        mode = LANDED;
      else
        break;
    case LANDED:
      //setLEDColor(247, 0, 255); //Purple
      landMillis = millis();
  }
  
  logLine[FLIGHT_MODE_LOG] = String(mode);
  //beginProcess();
  updateSD();
  //endProcess("SD Log update");

  //Blank space for each iteration
  //Serial.print("\n\n\n"); 

  //Plotting data for sim
  // serialTag("Y Estimate", getYEstimate());
  // serialTag("Apogee Estimate", getApogeeEstimate());
  // serialTag("Servo deployment", getServoDeployment() * 10);
  // serialTag("Acceleration z:", getVerticalAcceleration());
  // Serial.println();
  //delay(10);
  endProcess("Loop iteration");
}

void updateTime(){
  if(SIMULATION)
    timeNewMillis = simLine[0].toInt();
  else
    timeNewMillis = millis();
  dt = (timeNewMillis - timeMillis)/1000.0;
  timeMillis = timeNewMillis;
  logLine[0] = String(timeMillis);
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