//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
const int front_ultrasonic_pin = 6;
const int side_sensorPin = A3;
const int side_ultrasonic_pin = 7;
const int front_sensorPin = A2;

  
void setup() {
  Serial.begin(9600);
}


void loop() { 
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  AFMS.begin();
  Adafruit_DCMotor *left = AFMS.getMotor(1);
  Adafruit_DCMotor *right = AFMS.getMotor(2);
  
  delay(2000);
  back_to_mark_point(left, right, 0);
  
  }


void through_tunnel(Adafruit_DCMotor *left,Adafruit_DCMotor *right) { //done
  //from starting point - go into tunnel
  //Move to the tunnel entrance
  forward(left,right,96);
  
  //Facing the tunnel
  clockwise_90(left,right);
  
  //Move through tunnel to the mark-up point
  forward(left,right,117);
}


void back_to_mark_point(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int direction_count) {
  //from any location back to the mark point of the tunnel after picking up a victim
  direction_count %= 4;
  int turns = (direction_count - 1) % 4; //times of turning anticlockwise_90 needed to face the upper edge

  for (int i=0; i<turns; i++) {
    anticlockwise_90(left,right);
  }

  int distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);

  //move towards the upper edge of the wall
  forward(left, right, 50);
  backward(left, right, 5);
  anticlockwise_90(left,right);

  //move towards side edge
  distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  forward(left, right, 50); 
  delay(1000);
  //move back to mid line
  backward(left, right, 110); 
  anticlockwise_90(left,right); //facing tunnel
  forward(left, right, 89);
}


void tunnel_to_red_zone(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  //from mark point back through the tunnel to drop victim
  forward(left, right, 95);

  clockwise_90(left, right);

  forward(left, right, 70);
}


void red_to_tunnel(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  //back into the cave after drop the victim
  backward(left, right, 70);

  clockwise_90(left, right);

  forward(left, right, 95);
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


void adjust_wall(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  int ave_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  Serial.println(ave_distance);
  while (ave_distance>10) {
    forward_slowly(left,right);
    ave_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  }
    
  while (ave_distance<5) {
    backward_slowly(left,right);
    ave_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  }
 }

    
void forward(Adafruit_DCMotor *left, Adafruit_DCMotor *right,int distance){
  float wait_time;
  int i;
  left->run(FORWARD);
  right->run(FORWARD);
  for (i=0; i<200; i++) {
    left->setSpeed(i);
    right->setSpeed(i);  
    delay(10);
  }
  wait_time = (distance/1.89);
  left->setSpeed(200);  
  right->setSpeed(199);
  delay(wait_time*100);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
 }

void forward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right){
  //move forward with speed 50
  left->run(FORWARD);
  right->run(FORWARD);
  left->setSpeed(30);  
  right->setSpeed(32);
  delay(200);
  left->run(RELEASE);
  right->run(RELEASE);
}

void backward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right){
  //move backward with speed 50
  left->run(BACKWARD);
  right->run(BACKWARD);
  left->setSpeed(30);  
  right->setSpeed(32);
  delay(200);
  left->run(RELEASE);
  right->run(RELEASE);
}

void backward(Adafruit_DCMotor *left,Adafruit_DCMotor *right,int distance){
  //move backward with speed 200
  float wait_time;
  int i;
  left->run(BACKWARD);
  right->run(BACKWARD);
  for (i=0; i<200; i++) {
    left->setSpeed(i);
    right->setSpeed(i);  
    delay(10);
  }
  wait_time = (distance/1.89);
  left->setSpeed(200);  
  right->setSpeed(200);
  delay(wait_time*100);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

void clockwise_90(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  //turn clockwise 90 degrees
  left->run(FORWARD);
  right->run(BACKWARD);
  left->setSpeed(50);
  right->setSpeed(52); 
  delay(7100);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}


void anticlockwise_90(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  //turn anticlockwise 90 degrees
  left->run(BACKWARD);
  right->run(FORWARD);
  left->setSpeed(50);
  right->setSpeed(52); 
  delay(6790);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

int find_max(int a[]) {
  //find the maximum index in the array length of 5
  int max_num = a[0];
  for (int i = 0; i < 5; i++) {
    if (a[i] > max_num) {
      max_num = a[i];
    }
  }
  return max_num;
}

int find_min(int a[]) {
  //find the minimum index in the array length of 5
  int min_num = a[0];
  for (int i = 0; i < 5; i++) {
    if (a[i] < min_num) {
      min_num = a[i];
    }
  }
  return min_num;
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



bool side_search(Adafruit_DCMotor *left, Adafruit_DCMotor *right) { //done
  //search along the upper edge of the wall using side distance sensor
  int side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin,side_sensorPin);
  int front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  while (side_distance >140) {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(50);  
    right->setSpeed(52);
    side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  }
  if (side_distance < 100) {
    return true;
  }
  else {
    return false;
  }
}

void forward_till_obstacle(Adafruit_DCMotor *left, Adafruit_DCMotor *right) { //done
  //forward moving while detecting obstacle either victim or wall
  int distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  int new_speed = 50;
  while (distance > 13) {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(new_speed);  
    right->setSpeed(52);
    delay(500);
    distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  }
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}
