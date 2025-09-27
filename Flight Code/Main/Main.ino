 #define SERIAL true //false during flight
 #define SIMULATION true //false on real flight



//Shared pin definitions
#define SCK 13
#define MISO 12
#define MOSI 11

#define BURN_TIME 1.5
#define ALTIMETER_LOCKOUT 60 //altimeter delay (seconds)

//Logging variables for global scope convinience
#define ITEMS_LOGGED 12

//Header for log line. ** indicates fed in to sim
String logLine[] = {"**time stamp**",	"raw pressure",	"raw temperature", "**raw ax**",	"**raw ay**",	"**raw az**",	"**calculated alt**",	"y estimate",	"v estimate",	"a estimate",	"apogee estimate", "servo deployment"};
String simLine[ITEMS_LOGGED] = {};

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
    for(int i = 0; i < 5; i++){
      setTone(5000, 500);
      delay(1000);
    }
    delay(5000);
  }

  initializeServo();

  if(!SIMULATION){
    initializeIMU();

    setTone(1000, 1000);
    //delay(ALTIMETER_LOCKOUT * 1000);
    initializeAlt();
    setTone(1000, 3000);
  }


  setGreenLED(HIGH); //Signify successful
}

void loop() {
  if(SIMULATION)
    updateSim();
  
  updateTime();
  updateKalmanFilter();
  updateIMU();
  getTemperature();
  
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

  updateSD();
    


  //Serial.println(getYEstimate());
  delay(10);
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