#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); //left motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // right motor


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  AFMS.begin();
}


void ForwardToTunnel() {
  //move forward from the starting point to the tunnel entrance
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);

  myMotor1->setSpeed(200);  
  myMotor2->setSpeed(202);

  delay(5800); // to be adjusted

  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);
  delay(1000);
}


void Clockwise90() {
  //turning 90 degree clockwise
  myMotor1->setSpeed(50);
  myMotor2->setSpeed(52); 
  
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  delay(6300); // to be adjusted

  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);
  delay(1000);
}


void AntiClockwise90() {
  //turning 90 degree anti-clockwise
  myMotor1->setSpeed(50);
  myMotor2->setSpeed(52); 
  
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  delay(6300); // to be adjusted

  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);
  delay(1000);
}


void ThroughTunnel() {
  //enter the cave through the tunnel
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  
  myMotor1->setSpeed(200);  
  myMotor2->setSpeed(202);
  
  delay(6786); // to be adjusted

  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);
  delay(1000);
}


void loop() {
  int state = 1;
//  if (Serial.available()>0){
//  state = Serial.read();}
  
  if (bool (state == 1)){
    int i; // speed parameter

    ForwardToTunnel(); //forward from starting point
    Clockwise90(); //turning to the right

    //end
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    state = 0;
    delay(50000);
  }}
