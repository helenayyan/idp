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

  int direction_count = 0; //mark the direction facing to
  int victim_num = 0; //number of victims saved
  
  if (state == '1'){
    through_tunnel(left, right); //entering the tunnel
    direction_count += 1;
    
    //Move towards the wall --- could start searching from this point
    forward(left,right,89);
    adjust_wall(left,right);
  
    //Turn left
    anticlockwise_90(left,right);
    direction_count -= 1;
    
    //Move towards the wall
    forward(left,right,115);
    adjust_wall(left,right);
  
    //Turn left
    anticlockwise_90(left,right);
    direction_count -=1;
    
    //Move to the bottom left
    forward(left,right,150);

    //Turn left
    anticlockwise_90(left, right);
    direction_count -= 1;
    }
  }


void through_tunnel(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  //from starting point - go into tunnel
  //Move to the tunnel entrance
  forward(left,right,100);
  
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

  adjust_wall(left, right); //not sure?? //move towards the upper edge of the wall
  anticlockwise_90(left,right);
  adjust_wall(left, right);  //move towards side edge
  backward(left, right, 120); //move back to mid line
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


int ultra_sonic() {
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

void adjust_wall(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  int ave_distance = reliable_ultra_sonic_reading()
  while (ave_distance>20) {
    forward_slowly(left,right);
    ave_distance = reliable_ultra_sonic_reading();
  }
    
  while (ave_distance<20) {
    backward_slowly(left,right);
    ave_distance = reliable_ultra_sonic_reading();
  }
 }

    
void forward(Adafruit_DCMotor *left, Adafruit_DCMotor *right,int distance){
  //move forward with speed 200
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
  //move forward with speed 50
  left->run(FORWARD);
  right->run(FORWARD);
  left->setSpeed(30);  
  right->setSpeed(32);
  delay(100);
  left->run(RELEASE);
  right->run(RELEASE);
}

void backward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right){
  //move backward with speed 50
  left->run(BACKWARD);
  right->run(BACKWARD);
  left->setSpeed(50);  
  right->setSpeed(52);
  delay(100);
  left->run(RELEASE);
  right->run(RELEASE);
}

void backward(Adafruit_DCMotor *left,Adafruit_DCMotor *right,int distance){
  //move backward with speed 200
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

void clockwise_90(Adafruit_DCMotor *left,Adafruit_DCMotor *right) {
  //turn clockwise 90 degrees
  left->run(FORWARD);
  right->run(BACKWARD);
  left->setSpeed(50);
  right->setSpeed(52); 
  delay(6300);
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
  delay(6300);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}


int reliable_ultra_sonic_reading(){
  bool reliable = true;
  array distance[5] = {0,0,0,0,0};
  //reading average distance from ultrasonic sensor
  while (reliable){
  for (int i=0;i<5;i++){
    distance[i] = ultra_sonic();
    max_dist = max(distance);
    min_dist = min(distance);
    diff = max_dist - min_dist;
    if (diff <20){
      reliable = false}
    int sum_distance = 0
    for (int i=0;i<5;i++)
    {sum_distance +=distance[i];}
    int ave_distance = sum_distance/5;
    return ave_distance;
     }
