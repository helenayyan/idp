#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  AFMS.begin();
  
  // Set the speed to start, from 30 (off) to 255 (max speed)

  //myMotor1->run(FORWARD);
  //myMotor2->run(FORWARD);
  // turn on motor
  //myMotor1->run(RELEASE);
  //myMotor2->run(RELEASE);
}

void loop() {
//    int state;
//  if (Serial.available()>0){
//    state = Serial.read();}
//  
//  if (int state == 1){
//  Serial.print("Hello World");
  int i;
  
  Serial.print("tick");
  //test motor forward
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  
  for (i=0; i<200; i++) {
    myMotor1->setSpeed(i);  
    myMotor2->setSpeed(i+2);
    delay(10);
  }
delay(4500);

  for (i=200; i>0; i=i-5) {
    myMotor1->setSpeed(i);
    myMotor2->setSpeed(i+2); 
    delay(10);  }
    delay(3000);
    


  //test turning to the right
  Serial.print("clockwise");
  myMotor1->setSpeed(50);
  myMotor2->setSpeed(52); 
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  delay(6300);

  Serial.print("tech");
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  delay(1000);
  

  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  
  for (i=0; i<200; i=I+5) {
    myMotor1->setSpeed(i);  
    myMotor2->setSpeed(i+1);
    delay(10);
  }
delay(10000);
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);

  }
