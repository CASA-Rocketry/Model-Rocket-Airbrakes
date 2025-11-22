#define SIMULATION false //false on real flight
#define BUZZER true
#define SERVO true
#define SERIAL false//true//false during flight

//Remove print statements pre-compilation if SERIAL is false
#if SERIAL
  #define sPrint(a) Serial.print(a)
  #define sPrintln(a) Serial.println(a)
#else
  #define sPrint(a) 
  #define sPrintln(a)
#endif

//Shared pin definitions
#define SCK 13
#define MISO 12
#define MOSI 11
#define BUTTON 7

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
#define ALTIMETER_LOCKOUT 60 //altimeter delay (seconds)

unsigned long timeMillis = 0, timeNewMillis;
unsigned long launchMillis, landMillis;
unsigned long processMicros = 0;
int loopCount = 0;
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
  pinMode(BUTTON, INPUT);

  #if SIMULATION
    setLEDColor(255, 0, 0); //Red for sim
    initializeSim();
  #else
    setLEDColor(0, 255, 0); //Green for flight
    initializeServo();
    initializeIMU();
    initializeAlt();

    //Only do servo test and long delay if button isn't pressed
    if(digitalRead(BUTTON) == LOW){
      testServo();
      setTone(1000, 1000);
      for(int i = 0; i < ALTIMETER_LOCKOUT; i++){
        setTone(3000, 250);
        delay(500);
        setTone(1000, 250);
        delay(500);
     // delay(ALTIMETER_LOCKOUT * 1000);
      }
    }
    
    calibrateAlt();
  #endif

  //Successful calibration
  sPrintln("Successful calibration");
  setTone(1000, 3000);
  setGreenLED(HIGH);
}

void loop() {
  
  #if SIMULATION
    updateSim();
  #endif

  updateTime();
  updateKalmanFilter();
  updateIMU();
  getTemperature();
  
  //Manage flight states
  switch(mode){
    case LAUNCH_PAD:
      if (getVerticalAcceleration() > 20){
        mode = THRUST;
        launchMillis = timeMillis;
      } else
        break;
    case THRUST:
      if((timeMillis - launchMillis)/1000.0 > BURN_TIME)
        mode = COAST;
      else
        break;
    case COAST:
      runApogeeControl();
    //   if (getVEstimate() < 0)
    //     mode = RECOVERY;
    //   else
    //     break;
    // case RECOVERY:
    //   runTimeControl();
      if(getYEstimate() < 10)
        mode = LANDING;
      else 
        break;
    case LANDING:
      setServoDeployment(0);
      if(abs(getVEstimate()) < 0.2)
        mode = LANDED;
      else
        break;
    case LANDED:
      landMillis = millis();
  }
  
  logLine[FLIGHT_MODE_LOG] = String(mode);
  updateSD();

  // #if SERIAL
  //   timeLoop();
  // #endif

  if(loopCount++ % 10 == 0){
    flushFlightFile();
    // endProcess("1000 loop iterations");
    // beginProcess();
  }
}

void updateTime(){
  #if SIMULATION
    timeNewMillis = simLine[0].toInt();
  #else
    timeNewMillis = millis();
  #endif
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
  sPrintln(processName + ": " + durationMicro + " us");
}

//Update loop time every 1000 iterations
void timeLoop(){
  if(loopCount++ % 1000 == 0){
    endProcess("1000 loop iterations");
    beginProcess();
  }
}