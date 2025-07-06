#include <Servo.h>

Servo servo;

void setup() {
  // put your setup code here, to run once:
  servo.attach(3);
  pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  servo.write(0);
  digitalWrite(2, HIGH);
  delay(1000);
  servo.write(90);
  digitalWrite(2, LOW);
  delay(1000);
}
