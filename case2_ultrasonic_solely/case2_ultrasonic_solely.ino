//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
const int front_ultrasonic_pin = 6;
const int side_sensorPin = A3;
const int side_ultrasonic_pin = 7;
const int front_sensorPin = A2;
//Shall we let the robot to detect distance right after leaving the tunnel if less than a threshold(distance to wall) no obstacle

void setup() {
  Serial.begin(9600);
  }

  
void loop() {
  
  double value1 = ultra_sonic(front_ultrasonic_pin, front_sensorPin);
  //double value1 = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  Serial.println(value1);

}


int ultra_sonic(int pin_num, int sensorPin) { //done
  // select the input pin for the potentiometer
  pinMode(pin_num, OUTPUT);
  int sensorValue = 0;  // variable to store the value coming from the sensor 
  unsigned long pulse;
  
  digitalWrite(pin_num, HIGH); //send pulse 10us long to trigger sensor
  delay(0.01);
  digitalWrite(pin_num, LOW);
  pulse = pulseIn(sensorPin, HIGH); //Read pulse width from low to high to low
  sensorValue =  pulse / 58 ; // Divide by factor given by sensor data sheet
  delay(100);
  return sensorValue;
 }



int reliable_ultra_sonic_reading(int pin_num, int sensorPin) { //done
  //take average of the 5 readings from ultrasonic sensor
  
  bool irreliable = true;
  int distance[5] = {0,0,0,0,0};
  //reading average distance from ultrasonic sensor
  while (irreliable){
    for (int i=0;i<5;i++) {
      distance[i] = ultra_sonic(pin_num, sensorPin);
      int max_dist = find_max(distance);
      int min_dist = find_min(distance);
      int diff = max_dist - min_dist;
      if (diff <20) {
        irreliable = false;
      }
    int sum_distance = 0;
    for (int i=0;i<5;i++){
      sum_distance +=distance[i];
    }
    int ave_distance = sum_distance/5;
    return ave_distance;
    }
  }
}


int find_max(int a[]) { //done
  //find the maximum index in the array length of 5
  int max_num = a[0];
  for (int i = 0; i < 5; i++) {
    if (a[i] > max_num) {
      max_num = a[i];
    }
  }
  return max_num;
}

int find_min(int a[]) { //done
  //find the minimum index in the array length of 5
  int min_num = a[0];
  for (int i = 0; i < 5; i++) {
    if (a[i] < min_num) {
      min_num = a[i];
    }
  }
  return min_num;
}
