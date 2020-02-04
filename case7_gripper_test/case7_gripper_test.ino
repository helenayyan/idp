//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
const int front_ultrasonic_pin = 9;
const int side_ultrasonic_pin = 0;
const int buttonPin = 3;
//Shall we let the robot to detect distance right after leaving the tunnel if less than a threshold(distance to wall) no obstacle

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
 
}

void loop() {
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  AFMS.begin();
//  Adafruit_DCMotor *left = AFMS.getMotor(1);
//  Adafruit_DCMotor *right = AFMS.getMotor(2);
  Adafruit_DCMotor *front = AFMS.getMotor(1);
  gripper_down(front);
  gripper_up(front);


}


void gripper_up(Adafruit_DCMotor *front){
  front->run(BACKWARD);
  front->setSpeed(200);  
  delay(3000);
  front->run(RELEASE);
  delay(5000);
  }



void gripper_down(Adafruit_DCMotor *front){
  front->run(FORWARD);
  front->setSpeed(200);
  delay(4000);
  front->run(RELEASE);
  delay(5000);
  }
  
