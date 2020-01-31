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
  Adafruit_DCMotor *left = AFMS.getMotor(1);
  Adafruit_DCMotor *right = AFMS.getMotor(2);

  through_tunnel(left, right);
 
}

void through_tunnel(Adafruit_DCMotor *left,Adafruit_DCMotor *right) { //done
  //from starting point - go into tunnel
  //Move to the tunnel entrance
  forward(left,right,100);
  
  //Facing the tunnel
  clockwise_90(left,right);
  
  //Move through tunnel to the mark-up point
  forward(left,right,117);
}



void forward(Adafruit_DCMotor *left, Adafruit_DCMotor *right,int distance){
  float wait_time;
  wait_time = (distance/1.724);
  left->run(FORWARD);
  right->run(FORWARD);
  left->setSpeed(200);  
  right->setSpeed(202);
  delay(wait_time*100);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
  }

void clockwise_90(Adafruit_DCMotor *left,Adafruit_DCMotor *right){
  left->run(FORWARD);
  right->run(BACKWARD);
  left->setSpeed(50);
  right->setSpeed(52); 
  delay(6300);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
  }
