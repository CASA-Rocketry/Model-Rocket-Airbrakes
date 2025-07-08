//Shared pin definitions
#define SCK 13
#define MISO 12
#define MOSI 11

unsigned long time = 0, timeNew;
float dt;


// enum FlightMode {
//   LAUNCH_PAD = 0,
//   THRUST = 1,
//   COAST = 2,
//   RECOVERY = 3,
//   LAND = 4
// }

//enum FlightMode mode = LAUNCH_PAD;


void setup() {
  Serial.begin(9600);
  
  //Initialize hardware
  initializeLED();
  initializeAlt();
  initializeServo();
  initializeLog();
  initializeKalmanFilter();
}

void loop() {
  //Update timers
  timeNew = millis() / 1000.0;
  dt = timeNew - time;
  time = timeNew;

  updateKalmanFilter(dt);
  updateLogs();

  


  //Mange flight states
  // switch(mode){
  //   case LAUNCH_PAD:
  //     if (v_f > 1)
  //       mode++;
  //   case THRUST:
  //     //Add accelerometer data
  //   case COAST:
  //     //Add airbrake control for apogge
  //     if (v_f < 0)
  //       mode++;
  //   case RECOVER:
  //     //Add airbrake control for flight time
  //     if(v_f > -0.5 && v_f < 0.5)
  //       mode++;
  //   case LAND:
  //     servoCommand = 0;
  // }
  delay(50);

}