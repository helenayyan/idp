//Including necessary libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
//Shall we let the robot to detect distance right after leaving the tunnel if less than a threshold(distance to wall) no obstacle

void setup() {
  Serial.begin(9600);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  double value1 = find_distance_encoder_A1();
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
    Serial.println(sensorValue);
    Serial.println("----------------------------------");
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
    Serial.println(count_encoder);
    double distance = arc_length * count_encoder;
    Serial.println("------------------------------");
    Serial.println(distance);
    delay(500);
    

    return distance;}
