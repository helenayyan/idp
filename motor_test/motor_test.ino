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
  myMotor1->setSpeed(50);
  myMotor2->setSpeed(52);
    // turn on motor
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
    // turn off motor
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}

void loop() {
  Serial.print("Hello World");
  // or int?
  uint8_t i;
  
  Serial.print("tick");

  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  //Accelerating test
  for (i=0; i<200; i++) {
    myMotor1->setSpeed(i);  
    myMotor2->setSpeed(i+2);
    delay(10);
  }
  //Decelerating test
  for (i=200; i!=0; i--) {
    myMotor1->setSpeed(i); 
    myMotor2->setSpeed(i+2); 
    delay(10);
  }
  
  Serial.print("tock");

  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  for (i=0; i<200; i++) {
    myMotor1->setSpeed(i);
    myMotor2->setSpeed(i+2); 
    delay(10);
  }
  for (i=200; i!=0; i--) {
    myMotor1->setSpeed(i);
    myMotor1->setSpeed(i+2);    
    delay(10);
  }

  Serial.print("tech");
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  delay(1000);
}
