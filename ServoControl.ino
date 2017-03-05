/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#define NUM_SERVOS 6  // number of servos that exist in the hand
#define SERVO_MIN           // minimum angle the servo can reach 
#define SERVO_MAX           // maximum angle the servo can reach

Servo servo[NUM_SERVOS];
ros::NodeHandle  nh;

//Servo servo;

void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  int i=0;  
  for(i=0; i<NUM_SERVOS; i++)
  {
    
    //Serial.print(cmd_msg.data[i]);
    digitalWrite(13, LOW-digitalRead(13));  //toggle led
    servo[i].write(cmd_msg.data[i]); //set servo angle, should be from 0-180
    digitalWrite(13, HIGH-digitalRead(13));  //toggle led
  }  
  /*servo[0].write(cmd_msg.data[0]); //set servo angle, should be from 0-180
  servo[1].write(cmd_msg.data[1]); //set servo angle, should be from 0-180
  servo[2].write(cmd_msg.data[2]); //set servo angle, should be from 0-180
  servo[3].write(cmd_msg.data[3]); //set servo angle, should be from 0-180
  servo[4].write(cmd_msg.data[4]); //set servo angle, should be from 0-180
  servo[5].write(cmd_msg.data[5]); //set servo angle, should be from 0-180
    */
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo[0].attach(3);
  servo[1].attach(4);
  servo[2].attach(14);
  servo[3].attach(15);
  servo[4].attach(16);
  servo[5].attach(17);
  //servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
