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
  
}

void loop() {
  int state = 1;
//  if (Serial.available()>0){
//    state = Serial.read();}
  
  if (bool (state == 1)){
  Serial.print("Hello World");
  int i;
  
  Serial.println("tick");
  //test motor forward
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  

  myMotor1->setSpeed(200);  
  myMotor2->setSpeed(202);

  delay(5800);

  myMotor1->setSpeed(0);  
  myMotor2->setSpeed(0);
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
  
  myMotor1->setSpeed(200);  
  myMotor2->setSpeed(202);
  
  delay(6786);
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  state = 0;
  delay(50000);
  }}
