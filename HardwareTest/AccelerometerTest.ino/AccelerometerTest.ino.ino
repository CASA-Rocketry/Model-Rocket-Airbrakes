void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  double x = analogRead(0);
  double y = analogRead(1);
  double z = toAcceleration(analogRead(2));
  // Serial.print("x: ");
  // Serial.print(x);
  // Serial.print("y: ");
  // Serial.print(y);
  // Serial.print("z: ");
  Serial.println(z);
  delay(20);
}

//Converts analog reading to m/s^2 units
double toAcceleration(double analog){
  double voltage = analog / 1023 * 5;
  double power = voltage / 3.3;
  //return power;
  return (2*power - 1) * 3 * 9.8; //Number of g's
  //return 2*(analog / 1023 * 5 - 0.5) * 9.8;
}
