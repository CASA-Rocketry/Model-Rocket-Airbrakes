#include <Servo.h>

Servo servo;
double i = 0;

void setup() {
  servo.attach(9);
  // put your setup code here, to run once:

}

void loop() {
  servo.write(0);
  delay(5000);
  servo.write(100);
  delay(1000);
  //servo.write(random(135));
  //delay(600); // servo.write((sin(i) + 1)*135.0/2);
  // //delay(200);
  // delay(10);
  // i += PI / 100;

  // put your main code here, to run repeatedly:

}
