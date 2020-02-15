# IDP software documentation

<img src="https://i.loli.net/2020/02/15/RnDE4CWvXdGeTBb.jpg" style="zoom:50%;" />

## Introduction

Integrated design project Lent 2020 requires students to work in group to design a technology demonstrator robot to carry out a simplified cave rescue mission using Arduino Uno WIFI Rev2. This documentation introduces the use and design of the robot built by group L110.

During the competition, the robot successfully went through the tunnel and picked up one victim. However, the robot failed to travel back to the red zone due to the motor error, which happened unexpectedly due to the motor imbalance.

## Dependencies

- Install Arduino IDE from its official [website](https://www.arduino.cc/en/main/software).
- Install Adafruit Circuit Playground**, **Adafruit Motor Shield V2 Library and Arduino MegaAVR Boards.



## Software flow chart

<img src="https://i.loli.net/2020/02/16/JUh7NMtLIBkqmpi.png" style="zoom:67%;" />

## Code structure & Algorithm 

1. The software team agrees to use the function programming method. There are several prototype functions:
   - `forward(left_motor,right_motor,distance)`: going forward for a certain distance
   - `backward(left_motor,right_motor,distance)`: going backward for a certain distance
   - `clockwise_90(left_motor,right_motor)`: make a 90 degrees clockwise turning
   - `anticlockwise_90(left_motor,right_motor)`: make a 90 degrees anticlockwise turning
   - `ultra_sonic(pin_num,sensorPin)`: return distance reading from the ultrasonic sensor
   - `gripper_up(gripper_motor)`: push victim out using the gripper mechanism
   - `gripper_down(gripper_motor)`: collect victim using the gripper mechanism
2. Modular functions for different purposes are built on top of prototype functions and are added to the main loop where needed.
3. Comments on the algorithm:
   - Hard-code method is used for guiding the robot going through the tunnel and going back to the red/white zone.
   - Searching algorithm: a victim is detected when the side ultrasonic sensor reading is a lot less than the pre-measured distance to the wall at that position.
   - Ultrasonic sensor readings are not stable, `reliable_ultra_sonic_reading(pin_num,sensorPin)` function is built to take the average of 5 similar readings and exclude readings that are not reasonable.
   - The robot is programmed to hit walls several times during the searching process in order to calibrate its direction.
   - `Back_to_red_zone()` is achieved by hitting the top wall and side wall respectively so the robot is located at the top corner. Hard-code method can then be implanted to navigate the robot back to the red zone.
4. Full code can be found at: [**1.0_beta_final_update.ino**](https://github.com/helenayyan/idp/blob/master/1.0_beta_final_update/1.0_beta_final_update.ino). More test cases and historical releases can be found in the repository.

## System diagram 

<img src="https://i.loli.net/2020/02/16/qXDw5uaOSUL4sck.png" alt="Snipaste_2020-02-15_18-16-45" style="zoom:67%;" />

## Connection

1. Pin connections:

   ``` c++
   const int front_ultrasonic_pin = 6;
   const int front_sensorPin = A2;
   const int side_ultrasonic_pin = 7;
   const int side_sensorPin = A3;
   const int find_victum_led =  0;
   const int BlinkLed = 5;
   ```

   - Pin configuration for Arduino can be changed in `1.0_beta_final_update.ino`
   - The front ultrasonic sensor is connected to the Arduino using digital pin 6 and analogue input pin 2. It is mainly used to identify position by measuring the distance to the front wall.
   - The side ultrasonic sensor is connected to the Arduino using digital pin 7 and analogue input pin 3. It is mainly used to detect victim and identity position by measuring the distance to the side wall.
   - A LED is connected to the Arduino using digital pin 0 which will blink for 1 second after approaching the victim.
   - A blinking-LED circuit is connected to the Arduino using digital pin 5. The LED will blink continuously during the searching process.

2.  Other connections:

   - Two large DC motors(40RPM) and a small DC motor(18RPM) are connected to the Arduino through the Adafruit motor shield. Large motors are used to drive the robot and the small motor is used to drive the gripper mechanism.
   - An independent health detecting circuit is connected to the 5V power pin. The circuit is capable of detecting health conditions of victims without using the microcontroller ADC. The green LED will blink if the victim is healthy otherwise the red LED will blink which means that medical treatment is required.
   - USB connection between PC and Arduino is used for compiling while serial connection is not used throughout the searching process.
