#define DEBUG true

//Shared pin definitions
#define SCK 13
#define MISO 12
#define MOSI 11

#define BURN_TIME 1.5

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
  Serial.begin(9600);
  if(DEBUG)
    while(!Serial);
  
  //Initialize hardware
  initializeLED();
  initializeLog();
  initializeAlt();
  initializeIMU();
  initializeServo();
  //initializeKalmanFilter(); 
  setGreenLED(HIGH); //Signify successful
}

void loop() {
  //Update timers
  timeNewMillis = millis();
  dt = (timeNewMillis - timeMillis)/1000.0;
  timeMillis = timeNewMillis;

  //updateKalmanFilter(dt);
  updateLogs();
  updateAcceleration();
  Serial.println(getVerticalAcceleration());

  


  //Mange flight states
  switch(mode){
    case LAUNCH_PAD:
      if (getVerticalAcceleration() > 50){
        mode = THRUST;
        launchMillis = millis();
      }
      else
        break;
    case THRUST:
      if((timeMillis - launchMillis)/1000.0 > BURN_TIME)
        mode = COAST;
      else
        break;
    case COAST:
      runApogeeControl();
      if (getVEstimate() < 0)
        mode = RECOVERY;
      else
        break;
    case RECOVERY:
      runTimeControl();
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

  delay(50);
}