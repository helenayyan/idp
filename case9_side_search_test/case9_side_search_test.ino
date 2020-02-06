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
  Adafruit_DCMotor *front = AFMS.getMotor(3);
  
  
  char state = '1';

  int direction_count = 0; //mark the direction facing to
  int victim_num = 0; //number of victims saved
  
  if (state == '1'){
    delay(2000);
    through_tunnel(left, right);
    clockwise_90(left,right);  
    int distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
    anticlockwise_90(left,right);  
    //75
    direction_count += 1;
    //Move towards the wall --- could start searching from this point
    if (distance < 70){
      delay(3000);
      forward(left,right,distance-20);
      delay(5000);
      }
    else {
      forward(left,right,80);
      //backward_wall(left, right, some distance) //not sure if needed, need to test the sensor searching area
      distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin); //check if any victims close to wall
      if (distance < 105){ // parameter to be adjusted by reading when wall !!!
        backward_wall(left, right, 3);
        anticlockwise_
        forward(left, right, distance-20);
      }
      else{
        backward_wall(left, right, 3);
        anticlockwise_90(left,right);
        direction_count -= 1;
        forward(left,right,10);
        bool side_search_result = side_search(left,right);
        if (side_search_result == true) {
          distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
          forward(left, right, 4);
          anticlockwise_90(left,right);
          direction_count -= 1;
          forward(left,right, distance - 27);
          delay(1000);
          gripper_up(front);
          delay(5000);
        }
        else {
          forward(left, right, 10);
        }
      }
    }
    back_to_mark_point(left, right, direction_count);
    clockwise_90(left, right);
    forward(left, right, 60);
    gripper_down(front);
    delay(100000);
  }
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

int ultra_sonic(int pin_num, int sensorPin) {
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
  right->setSpeed(197);
  delay(wait_time*100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);

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
  right->setSpeed(197);
  delay(wait_time*100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

void backward_wall(Adafruit_DCMotor *left,Adafruit_DCMotor *right,int distance){
  //move backward with speed 200
  float wait_time;
  int i;
  left->run(BACKWARD);
  right->run(BACKWARD);
  wait_time = (distance/1.89);
  left->setSpeed(200);  
  right->setSpeed(197);
  delay(wait_time*100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}
void gripper_up(Adafruit_DCMotor *front){
  //to catch victim
  front->run(BACKWARD);
  front->setSpeed(200);  
  delay(2000);
  front->run(RELEASE);
  delay(5000);
  }

void gripper_down(Adafruit_DCMotor *front){
  //to release victim
  front->run(FORWARD);
  front->setSpeed(200);
  delay(1800);
  front->run(RELEASE);
  delay(5000);
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
  delay(7000);
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

int reliable_ultra_sonic_reading(int pin_num, int sensorPin) {
  //take average of the 5 readings from ultrasonic sensor
  
  bool irreliable = true;
  int distance[5] = {0,0,0,0,0};
  //reading average distance from ultrasonic sensor
  while (irreliable){
    for (int i=0;i<5;i++) {
      distance[i] = ultra_sonic(pin_num, sensorPin);
      
      }
      
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

bool side_search(Adafruit_DCMotor *left, Adafruit_DCMotor *right) { 
  //search along the upper edge of the wall using side distance sensor
  int side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin,side_sensorPin);
  int front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin,front_sensorPin);
  while (side_distance >= 70 and front_distance >= 5) {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(50);  
    right->setSpeed(48);
    side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  }
  if (side_distance < 70) {
    left -> run(RELEASE);
    right -> run(RELEASE);
    return true;
  }
  return false; // front reach the edge
}

void back_to_mark_point(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int direction_count) {
  //from any location back to the mark point of the tunnel after picking up a victim
  direction_count %= 4;
  int turns = (direction_count +3 ) % 4; //times of turning anticlockwise_90 needed to face the upper edge

  for (int i=0; i<turns; i++) {
    anticlockwise_90(left,right);
  }

  int distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);

  //move towards the upper edge of the wall
  forward(left, right, distance + 10);
  backward(left, right, 5);
  anticlockwise_90(left,right);

  //move towards side edge
  distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  forward(left, right, distance + 10); 
  delay(1000);
  //move back to mid line
  backward(left, right, 103); 
  anticlockwise_90(left,right); //facing red
  forward(left, right, 150);
}

void forward_till_obstacle(Adafruit_DCMotor *left, Adafruit_DCMotor *right) { 
  //forward moving while detecting obstacle either victim or wall
  int distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  while (distance > 13) {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(50);  
    right->setSpeed(48);
    delay(500);
    distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  }
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}
