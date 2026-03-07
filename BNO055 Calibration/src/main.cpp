#include <Arduino.h>
#include <Adafruit_BNO055.h>

#define p(x) Serial.print(x) //short hand for printing
#define pl(x) Serial.println(x)

#define BUTTON 4

Adafruit_BNO055 bno{0x29}; //Replace with adress, either 0x28 or 0x29
imu::Vector<3> acceleration;
uint8_t systemCalib, gyroCalib, magCalib, accCalib;

void printAcceleration();
void printCalibration();
void printOffsets();


void setup() {
  //Open serial and wait for connection
  Serial.begin(9600);
  while(!Serial)
    delay(100);

  if(!bno.begin()){
    while(true){
      Serial.println("Could not connect to IMU");
      delay(1000);
    }
  }

  //Axis remap
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);

  bno.setSensorOffsets({34, -41, 75, 467, -55, 25, -1, -2, 1, 1000, 1000});
  pinMode(BUTTON, INPUT);
}

void loop() {
  printAcceleration();
  p(" -------- ");
  printCalibration();
  pl();
  delay(100);
  if(digitalRead(BUTTON)){
    printOffsets();
    delay(10 * 1000);
  }
}


void printAcceleration(){
  acceleration = bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER);
  p("("); p(acceleration.x()); p(","); p(acceleration.y()); p(","); p(acceleration.z()); p(")");
}

void printCalibration(){
  bno.getCalibration(&systemCalib, &gyroCalib, &accCalib, &magCalib);
  p("system: "); p(systemCalib); p(", gyro: "); p(gyroCalib); p(", acc: "); p(accCalib); p(", mag: "); p(magCalib);
}

void printOffsets(){
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);

  p("Accelerometer: ");
  p(offsets.accel_offset_x); p(" ");
  p(offsets.accel_offset_y); p(" ");
  p(offsets.accel_offset_z); p(" ");

  p("\nGyro: ");
  p(offsets.gyro_offset_x); p(" ");
  p(offsets.gyro_offset_y); p(" ");
  p(offsets.gyro_offset_z); p(" ");

  p("\nMag: ");
  p(offsets.mag_offset_x); p(" ");
  p(offsets.mag_offset_y); p(" ");
  p(offsets.mag_offset_z); p(" ");

  p("\nAccel Radius: ");
  p(offsets.accel_radius);

  p("\nMag Radius: ");
  p(offsets.mag_radius);
  
  p("\n\n\n");
  p("{"); 
  p(offsets.accel_offset_x); p(", "); 
  p(offsets.accel_offset_y); p(", "); 
  p(offsets.accel_offset_z); p(", "); 
  p(offsets.mag_offset_x); p(", "); 
  p(offsets.mag_offset_y); p(", "); 
  p(offsets.mag_offset_z); p(", ");
  p(offsets.gyro_offset_x); p(", ");
  p(offsets.gyro_offset_y); p(", ");
  p(offsets.gyro_offset_z); p(", "); 
  p(offsets.accel_radius); p(", "); 
  p(offsets.accel_radius); pl("}");
}