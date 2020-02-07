//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//Define pins
const int front_ultrasonic_pin = 6;
const int side_sensorPin = A3;
const int side_ultrasonic_pin = 7;
const int front_sensorPin = A2;
const int buttonPin = 3;

const int ledPin =  LED_BUILTIN;// the number of the LED pin

// Variables will change:
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;

void setup()
{
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  //Initialise motors
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();
  AFMS.begin();
  Adafruit_DCMotor *left = AFMS.getMotor(1);
  Adafruit_DCMotor *right = AFMS.getMotor(2);
  Adafruit_DCMotor *front = AFMS.getMotor(3);
  // blink led
  unsigned long currentMillis = millis();
  
  static int victim_num = 0;  //number of victims saved

  while (victim_num < 4)
  {
    int direction_count = 0;
    bool holding_victum = false;
    
    if (victim_num == 0)
    {
      //Going through tunnel from start point
      delay(2000);
      through_tunnel(left, right);
      direction_count = 1;
    }
    
    else if (victim_num > 0 && victim_num < 4) 
    {
      red_to_tunnel(left, right);
      direction_count = 1;
    }

    //Detect whether there is a victim in the front
    clockwise_90(left, right);
    int distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
    anticlockwise_90(left, right);

    //If there is a victim right in front of the tunnel
    if (distance < 70)
    {
      approach_victim(left, right, front, distance);
      holding_victum = true;
    }

    //No victim in front of the tunnel start searching left region
    else
    {
      strike_front_wall(left, right);
      
      //ensure no victim close to the wall
      distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
      anticlockwise_90(left, right);
      direction_count -= 1;
      
      if (distance < 80)
      {
        approach_victim(left, right, front, distance);
        holding_victum = true;
      }
      // No victim along the wall, starting side search
      else
      {
        forward(left, right, 10);
        bool side_search_result = side_search_L(left, right);

        //If a victim is found
        if (side_search_result == true)
        {
          found_victim_L(left, right, front);
          direction_count -= 1;
          holding_victum = true;
        }

        //If no victim is found
        else
        {
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
      back_to_red(left, right, front);
      victim_num++;
      direction_count = 0;
      delay(5000);
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

      //If a victim is found
      if (side_search_result == true)
      {
        found_victim_R(left, right, front);
        direction_count -= 1;
        holding_victum = true;
      }
      else
      {
        backward(left, right, 15);
        forward(left, right, 4);
      }
    }

    back_to_mark_point(left, right, direction_count);

    if (holding_victum == true)
    {
      back_to_red(left, right, front);
      victim_num++;
      direction_count = 0;
      delay(5000);
    }
    else
    {
      //give up and go back to white
      forward(left, right, 155);
      anticlockwise_90(left, right);
      forward(left, right, 96);
      delay(500000);
    }
    delay(100000);
  }

  //find all four victims
  backward(left, right, 155);
  left->run(RELEASE);
  right->run(RELEASE);
}

void through_tunnel(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //done
  //from starting point - go into tunnel
  //Move to the tunnel entrance
  forward(left, right, 96);

  //Facing the tunnel
  clockwise_90(left, right);

  //Move through tunnel to the mark-up point
  forward(left, right, 120);
}

int ultra_sonic(int pin_num, int sensorPin)
{
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
  float wait_time;
  int i;
  left->run(FORWARD);
  right->run(FORWARD);
  for (i = 0; i < 200; i++)
  {
    left->setSpeed(i);
    right->setSpeed(i);
    delay(10);
  }

  wait_time = (distance / 1.89);
  left->setSpeed(200);
  right->setSpeed(197);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);

}

void forward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
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

  wait_time = (distance / 1.89);
  left->setSpeed(52);
  right->setSpeed(50);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);

}

void backward(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //move backward with speed 200
  float wait_time;
  int i;
  left->run(BACKWARD);
  right->run(BACKWARD);
  for (i = 0; i < 200; i++)
  {
    left->setSpeed(i);
    right->setSpeed(i);
    delay(10);
  }

  wait_time = (distance / 1.89);
  left->setSpeed(200);
  right->setSpeed(197);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

void backward_slowly(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
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

  wait_time = (distance / 1.89);
  left->setSpeed(52);
  right->setSpeed(50);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

void gripper_down(Adafruit_DCMotor *front)
{
  //to catch victim
  front->run(BACKWARD);
  front->setSpeed(200);
  delay(2000);
  front->run(RELEASE);
  delay(5000);
}

void gripper_up(Adafruit_DCMotor *front)
{
  //to release victim
  front->run(FORWARD);
  front->setSpeed(200);
  delay(1800);
  front->run(RELEASE);
  delay(5000);
}

void clockwise_90(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //turn clockwise 90 degrees
  left->run(FORWARD);
  right->run(BACKWARD);
  left->setSpeed(50);
  right->setSpeed(52);
  delay(7140);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

void anticlockwise_90(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //turn anticlockwise 90 degrees
  left->run(BACKWARD);
  right->run(FORWARD);
  left->setSpeed(50);
  right->setSpeed(52);
  delay(7100);
  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
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
  //search along the upper edge of the wall using side distance sensor
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
  //search along the upper edge of the wall using side distance sensor
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
  forward(left, right, distance + 10);
  delay(1000);
  //move back to mid line
  backward(left, right, 105);
  anticlockwise_90(left, right);  //facing red
}

void found_victim_L(Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front)
{
  int distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  forward_slowly(left, right, 5);
  anticlockwise_90(left, right);
  approach_victim(left, right, front, distance);
}

void found_victim_R(Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front)
{
  int distance = reliable_ultra_sonic_reading(side_ultrasonic_pin, side_sensorPin);
  backward_slowly(left, right, 5);
  anticlockwise_90(left, right);
  approach_victim(left, right, front, distance);
}

void backward_wall(Adafruit_DCMotor *left, Adafruit_DCMotor *right, int distance)
{
  //move backward with speed 200
  float wait_time;
  int i;
  left->run(BACKWARD);
  right->run(BACKWARD);
  wait_time = (distance / 1.89);
  left->setSpeed(200);
  right->setSpeed(197);
  delay(wait_time *100);

  left->run(RELEASE);
  right->run(RELEASE);
  delay(2000);
}

void red_to_tunnel(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //back into cave for more victims
  backward(left, right, 60);
  clockwise_90(left, right);
  forward(left, right, 98);
}

void approach_victim (Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front, int distance)
{
  //move toward victim when facing it and upload victim
  delay(1000);
  forward(left, right, distance-10);
  delay(1000);
  gripper_down(front);
  delay(1000);
}

void back_to_red(Adafruit_DCMotor *left, Adafruit_DCMotor *right, Adafruit_DCMotor *front)
{
  //after back to mark point, to drop victim to red zone
  forward(left, right, 155);
  clockwise_90(left, right);
  forward(left, right, 60);
  gripper_up(front);
}

void strike_front_wall(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
{
  //strike the front edge to adjust position
  forward(left, right, 80);
  backward_wall(left, right, 8);
}
