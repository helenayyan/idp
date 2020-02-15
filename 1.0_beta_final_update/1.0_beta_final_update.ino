//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//Define pins
const int front_ultrasonic_pin = 6;
const int side_sensorPin = A3;
const int side_ultrasonic_pin = 7;
const int front_sensorPin = A2;
const int BlinkLed = 5;
const int find_victum_led = 0;  // the number of the LED pin

void setup()
{
  Serial.begin(9600);
  pinMode(BlinkLed, OUTPUT);
}

void loop()
{
  //Initialise motors
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();
  AFMS.begin();
  Adafruit_DCMotor *left = AFMS.getMotor(1);
  Adafruit_DCMotor *right = AFMS.getMotor(2);
  Adafruit_DCMotor *front1 = AFMS.getMotor(3);

  //number of victims saved
  static int victim_num = 0;

  while (victim_num < 4)
  {
    //LED start to blink during searching process
    digitalWrite(BlinkLed, HIGH);
    //Direction the robot facing is saved
    int direction_count = 0;
    bool holding_victum = false;

    //Starting from the white zone at the start i.e. no victim is found
    if (victim_num == 0)
    {
      //Going through tunnel from start point
      delay(2000);
      through_tunnel(left, right);
      direction_count = 1;
    }

    //Starting from the red zone otherwise
    else if (victim_num > 0 && victim_num < 4)
    {
      //Going through the tunnel from red zone
      red_to_tunnel(left, right);
      direction_count = 1;
    }

    //Detect whether there is a victim in the front using the side sensor
    clockwise_90(left, right);
    direction_count += 1;
    int distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
    int distance_front = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
    backward_wall(left, right, 40);

    //Searching the area right in front of the tunnel
    bool side_search_result1 = side_search_front(left, right, distance_front);
    if (side_search_result1 == true)
    {
      //if a victim is found in front of the tunnel
      found_victim_L(left, right, front1);
      direction_count -= 1;
      holding_victum = true;
    }

    //No victim in front of the tunnel start searching left region
    else
    {
      backward_wall(left, right, 10);
      anticlockwise_90(left, right);
      direction_count = 1;

      //Hit the front wall to adjust direction
      strike_front_wall(left, right);

      //ensure no victim close to the wall
      distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
      anticlockwise_90(left, right);
      direction_count -= 1;

      if (distance < 60)
      {
        //if a victum is found along the wall
        approach_victim(left, right, front1, distance);
        holding_victum = true;
      }

      // No victim along the wall, starting side search
      else
      {
        forward(left, right, 10);
        bool side_search_result = side_search_L(left, right);

        if (side_search_result == true)
        {
          //if a victum is found in the left region
          found_victim_L(left, right, front1);
          direction_count -= 1;
          holding_victum = true;
        }
        else
        {
          //If no victim is found in the left region, hit the wall to adjust direction
          forward(left, right, 10);
          backward_wall(left, right, 5);
        }
      }
    }

    //Going back to the middle line of tunnel
    back_to_mark_point(left, right, direction_count);

    //If holding a victum, going back to the red region
    if (holding_victum == true)
    {
      back_to_red(left, right, front1);
      victim_num++;
      direction_count = 0;
      delay(1000);
    }

    //If not holding a victim, continue searching right side
    else
    {
      clockwise_90(left, right);
      clockwise_90(left, right);
      direction_count = 1;

      //Strike the wall to adjust position
      strike_front_wall(left, right);

      anticlockwise_90(left, right);
      direction_count--;

      //Adjust position, need to go backward due to position of side sensor
      backward(left, right, 10);
      bool side_search_result = side_search_R(left, right);

      if (side_search_result == true)
      {
        //If a victim is found in the right region
        found_victim_R(left, right, front1);
        direction_count -= 1;
        holding_victum = true;
      }
      else
      {
        //No victim is found in the right region, hit the wall to adjust direction
        backward(left, right, 10);
        forward(left, right, 4);
      }

      back_to_mark_point(left, right, direction_count);

      if (holding_victum == true)
      {
        //Go to the red region if holding a victim
        back_to_red(left, right, front1);
        victim_num++;
        direction_count = 0;
        delay(1000);
      }
      else
      {
        //give up and go back to white region as no vitim is found in both left and right region
        forward(left, right, 155);
        anticlockwise_90(left, right);
        forward(left, right, 96);
        delay(10000);
      }

      delay(1000);
    }
  }

  //After finding all four victims, go back to the white region form the red region
  backward(left, right, 190);
  anticlockwise_90(left, right);
  forward_slowly(left, right, 50);
  left->run(RELEASE);
  right->run(RELEASE);
  digitalWrite(BlinkLed, LOW);
  delay(100000);
}

void through_tunnel(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //from starting point - go through tunnel

  //Move to the tunnel entrance
  forward(left, right, 98);

  //Facing the tunnel
  clockwise_90(left, right);

  //Move through tunnel to the mark-up point
  forward(left, right, 120);
}

int ultra_sonic(int pin_num, int sensorPin)
{
  //Get ultrasonic sensor reading

  // select the input pin for the potentiometer
  pinMode(pin_num, OUTPUT);
  int sensorValue = 0;  // variable to store the value coming from the sensor 
  unsigned long pulse;

  digitalWrite(pin_num, HIGH);  //send pulse 10us long to trigger sensor
  delay(0.01);
  digitalWrite(pin_num, LOW);
  pulse = pulseIn(sensorPin, HIGH); //Read pulse width from low to high to low
  sensorValue = pulse / 58; // Divide by factor given by sensor data sheet
  delay(100);
  return sensorValue;
}

void forward(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //Function for DC motors to go forward

  float wait_time;
  int i;
  left->run(FORWARD);
  right->run(FORWARD);
  for (i = 0; i < 250; i++)
  {
    left->setSpeed(i);
    right->setSpeed(i);
    delay(10);
  }

  wait_time = (distance / 2.5);
  left->setSpeed(250);
  right->setSpeed(244);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(500);

}

void forward_victim(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //Function used to adjust position of robot when a vicim is found
  //Simply forward function excluding the uniform accelerating process

  float wait_time;
  int i;
  left->run(FORWARD);
  right->run(FORWARD);

  wait_time = (distance / 1.78);
  left->setSpeed(200);
  right->setSpeed(194);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(500);

}

void forward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //Functions for DC motor to go forward slowly

  float wait_time;
  int i;
  left->run(FORWARD);
  right->run(FORWARD);
  for (i = 0; i < 50; i++)
  {
    left->setSpeed(i);
    right->setSpeed(i);
    delay(10);
  }

  wait_time = (distance / 1.7);
  left->setSpeed(52);
  right->setSpeed(48);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(500);

}

void backward(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //Functions for DC motor to back forward

  float wait_time;
  int i;
  left->run(BACKWARD);
  right->run(BACKWARD);
  for (i = 0; i < 250; i++)
  {
    left->setSpeed(i);
    right->setSpeed(i);
    delay(10);
  }

  wait_time = (distance / 2.5);
  left->setSpeed(250);
  right->setSpeed(244);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(1000);
}

void backward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //Functions for DC motor to go backward slowly

  float wait_time;
  int i;
  left->run(BACKWARD);
  right->run(BACKWARD);
  for (i = 0; i < 50; i++)
  {
    left->setSpeed(i);
    right->setSpeed(i);
    delay(10);
  }

  wait_time = (distance / 1.8);
  left->setSpeed(52);
  right->setSpeed(50);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(1000);
}

void gripper_up(Adafruit_DCMotor *front1)
{
  //Function for the gripper mechanism to release victim at the red zone

  front1->run(BACKWARD);

  front1->setSpeed(200);
  delay(2 *1700);
  front1->run(RELEASE);
  delay(1000);
}

void gripper_down(Adafruit_DCMotor *front1)
{
  //Function for the gripper mechanism to catch vicim in the searching region

  front1->run(FORWARD);
  front1->setSpeed(200);
  delay(2 *1550);
  front1->run(RELEASE);
  delay(1000);
  front1->run(FORWARD);
  front1->setSpeed(200);
  delay(2 *200);
  front1->run(RELEASE);
  delay(1000);
}

void clockwise_90(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //Function for robot to turn clockwise 90 degrees
  left->run(FORWARD);
  right->run(BACKWARD);
  left->setSpeed(150);
  right->setSpeed(152);
  delay(1700);
  right->run(RELEASE);
  left->run(RELEASE);
  delay(500);
}

void anticlockwise_90(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //Function for robot to turn anticlockwise 90 degrees
  left->run(BACKWARD);
  right->run(FORWARD);
  left->setSpeed(150);
  right->setSpeed(152);
  delay(1750);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(500);
}

int find_max(int a[])
{
  //find the maximum index in the array length of 5
  int max_num = a[0];
  for (int i = 0; i < 5; i++)
  {
    if (a[i] > max_num)
    {
      max_num = a[i];
    }
  }

  return max_num;
}

int find_min(int a[])
{
  //find the minimum index in the array length of 5
  int min_num = a[0];
  for (int i = 0; i < 5; i++)
  {
    if (a[i] < min_num)
    {
      min_num = a[i];
    }
  }

  return min_num;
}

int reliable_ultra_sonic_reading(int pin_num, int sensorPin)
{
  //Find the reliable ultrasonic reading and get rid of readings that are unreasonable

  //take average of the 5 readings from ultrasonic sensor
  bool irreliable = true;
  int distance[5] = { 0, 0, 0, 0, 0 };
  //reading average distance from ultrasonic sensor
  while (irreliable)
  {
    for (int i = 0; i < 5; i++)
    {
      distance[i] = ultra_sonic(pin_num, sensorPin);
    }

    int max_dist = find_max(distance);
    int min_dist = find_min(distance);
    int diff = max_dist - min_dist;
    if (diff < 20)
    {
      irreliable = false;
    }

    int sum_distance = 0;
    for (int i = 0; i < 5; i++)
    {
      sum_distance += distance[i];
    }

    int ave_distance = sum_distance / 5;
    return ave_distance;
  }
}

bool side_search_L(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //search along the left upper edge of the wall using side distance sensor

  int side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  int front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  while (side_distance >= 95 and front_distance >= 10)
  {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(50);
    right->setSpeed(48);
    side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
    front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  }

  if (side_distance < 95)
  {
    left->run(RELEASE);
    right->run(RELEASE);
    return true;
  }

  return false; // front reach the edge
}

bool side_search_R(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //search along the right upper edge of the wall using side distance sensor

  int side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  int front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  while (side_distance >= 95 and front_distance <= 210)
  {
    left->run(BACKWARD);
    right->run(BACKWARD);
    left->setSpeed(50);
    right->setSpeed(48);
    side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
    front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  }

  if (side_distance < 95)
  {
    left->run(RELEASE);
    right->run(RELEASE);
    return true;
  }

  return false; // front reach the edge
}

bool side_search_front(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int origin_front)
{
  //search along the exit of the tunnel using side distance sensor
  int side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  int front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  while (side_distance >= 40 and front_distance >= origin_front)
  {
    left->run(FORWARD);
    right->run(FORWARD);
    left->setSpeed(50);
    right->setSpeed(48);
    side_distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
    front_distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  }

  if (side_distance < 40)
  {
    left->run(RELEASE);
    right->run(RELEASE);
    return true;
  }

  return false; // front reach the edge
}

void back_to_mark_point(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int direction_count)
{
  //from any location back to the mark point of the tunnel after picking up a victim

  direction_count %= 4;
  int turns = (direction_count + 3) % 4;  //times of turning anticlockwise_90 needed to face the upper edge

  for (int i = 0; i < turns; i++)
  {
    anticlockwise_90(left, right);
  }

  int distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);

  //move towards the upper edge of the wall
  forward(left, right, distance + 10);
  backward(left, right, 5);
  anticlockwise_90(left, right);

  //move towards side edge
  distance = reliable_ultra_sonic_reading(front_ultrasonic_pin, front_sensorPin);
  forward(left, right, distance + 20);
  delay(500);
  //move back to mid line
  backward(left, right, 95);
  anticlockwise_90(left, right);  //facing red
}

void found_victim_L(Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front1)
{
  //Function to approach victims after detected by the ultrasonic sensor in the left region
  int distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  if (distance < 65)
  {
    forward_victim(left, right, 7);
  }
  else if (distance < 30)
  {
    forward_victim(left, right, 3);
  }
  else
  {
    forward_victim(left, right, 10);
  }

  anticlockwise_90(left, right);
  approach_victim(left, right, front1, distance);
}

void found_victim_R(Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front1)
{
  //Function to approach victims after detected by the ultrasonic sensor in the right region

  int distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  if (distance < 65)
  {
    forward_victim(left, right, 13);
  }
  else
  {
    forward_victim(left, right, 16);
  }

  anticlockwise_90(left, right);
  approach_victim(left, right, front1, distance);
}

void backward_wall(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //Move backward with speed 200 excluding the uniform accelerating process

  float wait_time;
  int i;
  left->run(BACKWARD);
  right->run(BACKWARD);
  wait_time = (distance / 1.78);
  left->setSpeed(200);
  right->setSpeed(194);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(1000);
}

void red_to_tunnel(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //Going back to the cave from the red region for more victum

  backward_wall(left, right, 10);
  anticlockwise_90(left, right);
  forward(left, right, 30);
  backward_wall(left, right, 5);
  clockwise_90(left, right);
  forward(left, right, 50);
  backward(left, right, 93);
  clockwise_90(left, right);
  forward(left, right, 130);
}

void approach_victim(Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front1, int distance)
{
  //move toward victim when facing it and upload victim

  delay(1000);
  forward_victim(left, right, distance + 3);
  delay(1000);

  digitalWrite(find_victum_led, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);  // wait for a second
  digitalWrite(find_victum_led, LOW); // turn the LED off by making the voltage LOW

  gripper_down(front1);
  delay(1000);
}

void back_to_red(Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front1)
{
  //After back to mark point(in front of the tunnel), to drop victim to red zone
  forward(left, right, 170);
  clockwise_90(left, right);
  forward(left, right, 55);
  gripper_up(front1);
}

void strike_front_wall(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //strike the front edge to adjust position
  forward(left, right, 80);
  backward_wall(left, right, 8);
}
