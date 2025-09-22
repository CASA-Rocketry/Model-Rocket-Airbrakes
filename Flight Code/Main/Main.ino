 #define SERIAL true //false during flight
 #define SIMULATION true //false on real flight
 #define SIMULATION_FULL_REPLAY false //false on real flight or when tuning through sim


//Shared pin definitions
#define SCK 13
#define MISO 12
#define MOSI 11

#define BURN_TIME 1.5
#define ALTIMETER_LOCKOUT 60 //altimeter delay (seconds)

//Logging variables for global scope convinience
#define ITEMS_LOGGED 8

String logLine[] = {"time stamp", "raw pressure", "raw ax", "raw ay", "raw az", "calibrated alt", "estimated apogee", "servo deployment"};

unsigned long timeMillis = 0, timeNewMillis;
unsigned long launchMillis, landMillis;
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
  if (!SIMULATION){
    setLEDColor(0, 255, 0); //Green for flight
    initializeSim();
  }
  else
    setLEDColor(255, 0, 0); //Green for sim replay


  //Initialize hardware
  initializeLog(); //If sim, this opens the sim file
  initializeLED();

  for(int i = 0; i < 5; i++){
      setTone(5000, 500);
      delay(1000);
    }

  delay(5000);

  initializeServo();

  if(!SIMULATION){
    //Warning tones
    
    initializeIMU();
    delay(ALTIMETER_LOCKOUT * 1000);
    initializeAlt();
  }

  

  setGreenLED(HIGH); //Signify successful
  //setServoDeployment(0.05);
}

void loop() {
  if(SIMULATION)
    updateSim();
  
  //Update timers
  if(SIMULATION)
    timeNewMillis = getSimMillis();
  else
    timeNewMillis = millis();
  dt = (timeNewMillis - timeMillis)/1000.0;
  timeMillis = timeNewMillis;
  logLine[0] = String(timeMillis);

  updateKalmanFilter();
  updateIMU();

  if(SIMULATION)
    setServoDeployment(getSimServoDeployment());
  //Serial.println(simServoDeployment());
  


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

  if(!SIMULATION)
    updateSD();


  Serial.println(getYEstimate());
  delay(10);
}