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
  Adafruit_DCMotor *front = AFMS.getMotor(3);
  gripper_down(front);
  gripper_up(front);
}


void gripper_up(Adafruit_DCMotor *front){
  //to catch victim
  front->run(BACKWARD);
  front->setSpeed(200);  
  delay(3000);
  front->run(RELEASE);
  delay(5000);
  }


void gripper_down(Adafruit_DCMotor *front){
  //to release victim
  front->run(FORWARD);
  front->setSpeed(200);
  delay(4000);
  front->run(RELEASE);
  delay(5000);
  }
  
