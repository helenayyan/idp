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
    int direction_count = 0;   //mark the direction facing to
    int victim_num = 0;   //number of victims saved
    Serial.println("start");
    
    through_tunnel(left, right); //entering the tunnel
    direction_count += 1;
    
    //Move towards the wall --- could start searching from this point
    forward_till_obstacle(left,right);
    adjust_wall(left,right);
  
    //Turn left
    anticlockwise_90(left,right);
    direction_count -= 1;

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

void back_to_mark_point(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int direction_count) { //Not done yet!!!
  //from any location back to the mark point of the tunnel after picking up a victim
  direction_count %= 4;
  int turns = (direction_count - 1) % 4; //times of turning anticlockwise_90 needed to face the upper edge

  for (int i=0; i<turns; i++) {
    anticlockwise_90(left,right);
  }

  int distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  //move towards the upper edge of the wall
  //Need to face upper wall
  forward(left, right, distance);
  adjust_wall(left, right); 
  anticlockwise_90(left,right);

  //move towards side edge
  distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  forward(left, right, distance);
  adjust_wall(left, right);  

  //move back to mid line
  backward(left, right, 120); 
  anticlockwise_90(left,right); //facing tunnel
  forward(left, right, 89);
}

void tunnel_to_red_zone(Adafruit_DCMotor *left,Adafruit_DCMotor *right) { //done
  //from mark point back through the tunnel to drop victim
  forward(left, right, 95);

  clockwise_90(left, right);

  forward(left, right, 70);
}

void red_to_tunnel(Adafruit_DCMotor *left,Adafruit_DCMotor *right) { //done
  //back into the cave after drop the victim
  backward(left, right, 70);

  clockwise_90(left, right);

  forward(left, right, 95);
}

void adjust_wall(Adafruit_DCMotor *left,Adafruit_DCMotor *right) { //Not done yet!!!
  int ave_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  while (ave_distance>20) {
    forward_slowly(left,right);
    ave_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  }
    
  while (ave_distance<20) {
    backward_slowly(left,right);
    ave_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  }
 }
    
void forward(Adafruit_DCMotor *left, Adafruit_DCMotor *right,int distance){ //done
  //move forward with speed 200 without victim searching
  float initial_distance = find_distance_encoder_A1();
  float distance_covered = 0;
  int new_speed = 200;
  while (distance_covered <= distance){
  left->run(FORWARD);
  right->run(FORWARD);
  left->setSpeed(new_speed);  
  right->setSpeed(202);
  new_speed = adjust_velocity(new_speed);
  distance_covered = find_distance_encoder_A1() - initial_distance;
  }
}

void forward_till_obstacle(Adafruit_DCMotor *left, Adafruit_DCMotor *right) { //done
  //forward moving while detecting obstacle either victim or wall
  int distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  int new_speed = 50;
  while (distance > 10) {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(new_speed);  
    right->setSpeed(52);
    new_speed = adjust_velocity(new_speed);
    distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  }
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

bool side_search(Adafruit_DCMotor *left, Adafruit_DCMotor *right) { //done
  //search along the upper edge of the wall using side distance sensor
  int side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin);
  int front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
  int new_speed = 50;
  while (side_distance >140 && front_distance > 10) {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(new_speed);  
    right->setSpeed(52);
    side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin);
    front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin);
    new_speed = adjust_velocity(new_speed);
  }
  if (side_distance < 140) {
    return true;
  }
  else {
    return false;
  }
}

void forward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right){ // May not need to use encoder
  //move forward with speed 30
  left->run(FORWARD);
  right->run(FORWARD);
  left->setSpeed(30);  
  right->setSpeed(32);
  delay(100);
  left->run(RELEASE);
  right->run(RELEASE);
}

void backward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right){ // May not need to use encoder
  //move backward with speed 50
  left->run(BACKWARD);
  right->run(BACKWARD);
  left->setSpeed(30);  
  right->setSpeed(32);
  delay(100);
  left->run(RELEASE);
  right->run(RELEASE);
}

void backward(Adafruit_DCMotor *left,Adafruit_DCMotor *right,int distance){ //done
  //move forward with speed 200 without victim searching
  float initial_distance = find_distance_encoder_A1();
  float distance_covered = 0;
  int new_speed = 200;
  while (distance_covered <= distance){
  left->run(BACKWARD);
  right->run(BACKWARD);
  left->setSpeed(new_speed);  
  right->setSpeed(202);
  new_speed = adjust_velocity(new_speed);
  distance_covered = find_distance_encoder_A1() - initial_distance;
  }
}


void clockwise_90(Adafruit_DCMotor *left,Adafruit_DCMotor *right) { //done
  //turn clockwise 90 degrees
  float initial_distance = find_distance_encoder_A1();
  float distance_covered = 0;
  while (distance_covered < 39.3){
  left->run(FORWARD);
  right->run(RELEASE);
  left->setSpeed(50); 
  distance_covered = find_distance_encoder_A1() - initial_distance;
  }
  
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

void anticlockwise_90(Adafruit_DCMotor *left,Adafruit_DCMotor *right) { //done
  //turn clockwise 90 degrees
  float initial_distance = find_distance_encoder_A2();
  float distance_covered = 0;
  while (distance_covered < 39.3){
  left->run(RELEASE);
  right->run(FORWARD);
  right->setSpeed(50); 
  distance_covered = find_distance_encoder_A2() - initial_distance;
  }
  
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
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

int ultra_sonic(int pin_num) { //done
  int sensorPin = A0;    // select the input pin for the potentiometer
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
 
int reliable_ultra_sonic_reading(int pin_num) { //done
  //take average of the 5 readings from ultrasonic sensor
  
  bool irreliable = true;
  int distance[5] = {0,0,0,0,0};
  //reading average distance from ultrasonic sensor
  while (irreliable){
    for (int i=0;i<5;i++) {
      distance[i] = ultra_sonic(pin_num);
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


double find_distance_encoder_A1(){ //done
    //left motor encoder
    int encoderPin = A1;
    static int old_value_sensor = 0; //stores old value of sensor reading
    static int count_encoder = 0; //total count
    int th_high = 73; //higher threshold of sensor output indicating white strip reached
    int th_low = 53; //lower threshold of sensor output indicating black strip reached
    float arc_length = 0.4433; //arc length at radius of wheel in cm (with 72 segments at radius 5.08cm, arc length = 5*(2*pi/72))

    int sensorValue = analogRead(encoderPin);  // variable to store the value coming from the sensor
    if (sensorValue >= th_high and old_value_sensor < th_high) {
        //checks that old_value is below the higher threshold and that the new value is above the threshold, indicating the sensor has gone from black to white
        count_encoder ++;
        old_value_sensor = sensorValue; //sets old_value to the current sensor value ready for the next loop
    }
    else if (sensorValue <= th_low and count_encoder == 0) {
        //checks that the old_value is above the lower threshold and that the new value is below the threshold, indicating the sensor has gone from white to black
        count_encoder ++;
        old_value_sensor = sensorValue; //sets old_value to the current sensor value ready for the next loop
    }
    else if (sensorValue <= th_low and old_value_sensor > th_low) {
        //checks that the old_value is above the lower threshold and that the new value is below the threshold, indicating the sensor has gone from white to black
        count_encoder ++;
        old_value_sensor = sensorValue; //sets old_value to the current sensor value ready for the next loop
    }
    double distance = arc_length * count_encoder;

    return distance;
}


double find_distance_encoder_A2(){ //done
    int encoderPin = A2;
    static int old_value_sensor = 0; //stores old value of sensor reading
    static int count_encoder = 0; //total count
    int th_high = 73; //higher threshold of sensor output indicating white strip reached
    int th_low = 53; //lower threshold of sensor output indicating black strip reached
    float arc_length = 0.4433; //arc length at radius of wheel in cm (with 72 segments at radius 5.08cm, arc length = 5*(2*pi/72))

    int sensorValue = analogRead(encoderPin);  // variable to store the value coming from the sensor
    if (sensorValue >= th_high and old_value_sensor < th_high){
        //checks that old_value is below the higher threshold and that the new value is above the threshold, indicating the sensor has gone from black to white
        count_encoder ++;
        old_value_sensor = sensorValue; //sets old_value to the current sensor value ready for the next loop
    }
    else if (sensorValue <= th_low and count_encoder == 0){
        //checks that the old_value is above the lower threshold and that the new value is below the threshold, indicating the sensor has gone from white to black
        count_encoder ++;
        old_value_sensor = sensorValue; //sets old_value to the current sensor value ready for the next loop
    }
    else if (sensorValue <= th_low and old_value_sensor > th_low){
        //checks that the old_value is above the lower threshold and that the new value is below the threshold, indicating the sensor has gone from white to black
        count_encoder ++;
        old_value_sensor = sensorValue; //sets old_value to the current sensor value ready for the next loop
    }

    double distance = arc_length*count_encoder;

    return distance;
}


double find_velocity_encoder_A1(){ //done
  //return the velocity of the left wheel
  double distance1 = find_distance_encoder_A1();
  int time1 = millis();
  delay(100);
  
  double distance2 = find_distance_encoder_A1();
  int time2 = millis();
  double velocity = (distance2-distance1)/(time2-time1);
  velocity *= 10;

  return velocity;
}

double find_velocity_encoder_A2(){ //done
  //return the velocity of the right wheel
  double distance1 = find_distance_encoder_A2();
  int time1 = millis();
  delay(100);
  
  double distance2 = find_distance_encoder_A2();
  int time2 = millis();
  double velocity = (distance2-distance1)/(time2-time1);
  velocity *= 10;
  return velocity;
}

int adjust_velocity(int left_speed){ //done
  //make the robot go straight by adjusting the left wheel
  double velocity1 = find_velocity_encoder_A1();
  double velocity2 = find_velocity_encoder_A2();
  double difference = velocity1 - velocity2;
  if (difference<-0.01){
    left_speed += 1;
  }
  else if (difference > 0.01){
    left_speed -= 1;
  }
  return left_speed;
}
