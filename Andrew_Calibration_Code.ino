#include <Servo.h>

Servo firstESC, secondESC, thirdESC, fourthESC; //Create as much as Servo object you want. You can controll 2 or more Servos at the same time

int value = 1000; // set values you need to zero

void setup() {
  Serial.begin(9600);    // start serial at 9600 baud

  firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(8);
  thirdESC.attach(7);
  fourthESC.attach(6);
}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
  Serial.println(value);
  if(Serial.available()) 
    value = Serial.parseInt();    // Parse an Integer from Serial
  firstESC.writeMicroseconds(value);
  secondESC.writeMicroseconds(value);
  thirdESC.writeMicroseconds(value);
  fourthESC.writeMicroseconds(value);
}

