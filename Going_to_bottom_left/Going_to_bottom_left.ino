//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


  
void setup() {
  Serial.begin(9600);
  
}

void loop() { 
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  AFMS.begin();
  Adafruit_DCMotor *left = AFMS.getMotor(1);
  Adafruit_DCMotor *right = AFMS.getMotor(2);
  
  
  char state = '0';
  while (Serial.available()>0){
  state = Serial.read();
  }
  
  if (state == '1'){
  //Move to the tunnel entrance
  forward(left,right,100);
  
  //Facing the tunnel
  clockwise_90(left,right);
  
  //Move through tunnel to the mark-up point
  forward(left,right,117);
  
  //Move towards the wall
  forward(left,right,89);
  adjust_wall(left,right);
  
  
  //Turn left
  anticlockwise_90(left,right);
  
  //Move towards the wall
  forward(left,right,115);
  adjust_wall(left,right);
  
  //Turn left
  anticlockwise_90(left,right);
  
  //Move to the bottom left
  forward(left,right,150);
  }
  }



int ultra_sonic(){
  int sensorPin = A0;    // select the input pin for the potentiometer
  pinMode(9, OUTPUT);
  int sensorValue = 0;  // variable to store the value coming from the sensor 
  unsigned long pulse;
  
  digitalWrite(9, HIGH); //send pulse 10us long to trigger sensor
  delay(0.01);
  digitalWrite(9, LOW);
  pulse = pulseIn(sensorPin, HIGH); //Read pulse width from low to high to low
  sensorValue =  pulse / 58 ; // Divide by factor given by sensor data sheet
  delay(100);
  return sensorValue;
  }

void adjust_wall(Adafruit_DCMotor *left,Adafruit_DCMotor *right){
  int distance = 0;
  for (int i=0;i<5;i++){
    distance +=ultra_sonic();}

  distance /=5;
  while (distance>20){
    forward_slowly(left,right);
    int distance = 0;
    for (int i=0;i<5;i++){
    distance +=ultra_sonic();}
    distance /=5;
    }
    

  while (distance<10){
    backward_slowly(left,right);
    int distance = 0;
    for (int i=0;i<5;i++){
    distance +=ultra_sonic();}
    distance /=5;
    } 
    }
    
void forward(Adafruit_DCMotor *left, Adafruit_DCMotor *right,int distance){
  float wait_time;
  wait_time = (distance/1.724);
  left->run(FORWARD);
  right->run(FORWARD);
  left->setSpeed(200);  
  right->setSpeed(202);
  delay(wait_time);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
  }

void forward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right){
  left->run(FORWARD);
  right->run(FORWARD);
  left->setSpeed(30);  
  right->setSpeed(32);
  delay(100);
  left->run(RELEASE);
  right->run(RELEASE);
  }

void backward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right){
  left->run(BACKWARD);
  right->run(BACKWARD);
  left->setSpeed(50);  
  right->setSpeed(52);
  delay(100);
  left->run(RELEASE);
  right->run(RELEASE);
  }

void backward(Adafruit_DCMotor *left,Adafruit_DCMotor *right,int distance){
  float wait_time;
  wait_time = (distance/1.724);
  left->run(BACKWARD);
  right->run(BACKWARD);
  left->setSpeed(200);  
  right->setSpeed(202);
  delay(wait_time);
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


void anticlockwise_90(Adafruit_DCMotor *left,Adafruit_DCMotor *right){
  left->run(BACKWARD);
  right->run(FORWARD);
  left->setSpeed(50);
  right->setSpeed(52); 
  delay(6300);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
  }
