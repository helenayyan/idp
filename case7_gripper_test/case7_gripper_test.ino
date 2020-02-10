//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
//Shall we let the robot to detect distance right after leaving the tunnel if less than a threshold(distance to wall) no obstacle

void setup() {
  Serial.begin(9600);
}

void loop() {
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  AFMS.begin();
  Adafruit_DCMotor *front1 = AFMS.getMotor(3);
//  Adafruit_DCMotor *front2 = AFMS.getMotor(4);
  gripper_down(front1);
  gripper_up(front1);
}


void gripper_up(Adafruit_DCMotor *front1){
  //to catch victim
  front1->run(FORWARD);
  //front2->run(BACKWARD);
//  front2->setSpeed(200);
  front1->setSpeed(200);   
  delay(2*1600);
  front1->run(RELEASE);
 // front2->run(RELEASE);
  delay(5000);
  }


void gripper_down(Adafruit_DCMotor *front1){
  //to release victim
  front1->run(BACKWARD);
  //front2->run(FORWARD);
  //front2->setSpeed(200);
  front1->setSpeed(200);   
  delay(2*1550);
  front1->run(RELEASE);
  //front2->run(RELEASE);
  delay(5000);
  }
  
