//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define BlinkLed  5 //pin for blinking led

const int ledPin =  0;// the number of the LED pin


void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(BlinkLed, OUTPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(BlinkLed, HIGH); //hold blinking led high
    delay(5000);
}
