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
#include<rosserial_arduino/Adc.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#define NUM_SERVOS 6  // number of servos that exist in the hand
#define SERVO_MIN           // minimum angle the servo can reach 
#define SERVO_MAX           // maximum angle the servo can reach
#define POT1 A0
#define POT2 A1
#define POT3 A2
#define POT4 A3
#define POT5 A4
#define POT6 A5
#define POT7 A6
#define POT8 A7
#define POT9 A8
#define POT10 A9
#define FSR1 A10
#define FSR2 A11
#define FSR3 A12
#define FSR4 A13
#define FSR5 A14
#define BAUD 9600

Servo servo[NUM_SERVOS];
ros::NodeHandle  nh;

//Servo servo;
std_msgs::UInt16 msg_pub;
std_msgs::UInt16 msg_pub1;

std_msgs::Int16MultiArray pot_array_raw; //Raw POT values from ADC
//pot_array_raw.layout.dim.push_back(std_msgs::MultiArrayDimension());
//pot_array_raw.layout.dim[0].size = 9;

std_msgs::Int16MultiArray pot_array;     //Processed POT Angles (in degrees)
//pot_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
//pot_array.layout.dim[0].size = 9;

std_msgs::Int16MultiArray fsr_array_raw;         //Raw FSR values from ADC
//fsr_array_raw.layout.dim.push_back(std_msgs::MultiArrayDimension());
//fsr_array_raw.layout.dim[0].size = 5;

std_msgs::Int16MultiArray fsr_array;             //Processed FSR Forces (in grams)
//fsr_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
//fsr_array.layout.dim[0].size = 5;

ros::Publisher chatter_pub("servo_pub", &msg_pub);
ros::Publisher chatter_pub1("sensor_pub", &msg_pub1);
ros::Publisher pot_pub("sensors/potentiometer_raw", &pot_array_raw);
ros::Publisher pot_proc("sensors/potentiometer_processed", &pot_array);
ros::Publisher fsr_pub("sensors/fsr_raw", &fsr_array_raw);
ros::Publisher fsr_proc("sensors/fsr_processed", &fsr_array);

void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  int i=0;  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
  for(i=0; i<NUM_SERVOS; i++)
  {
    msg_pub.data = cmd_msg.data[i];
    chatter_pub.publish(&msg_pub);
    //Serial.print(cmd_msg.data[i]);
    servo[i].write(cmd_msg.data[i]); //set servo angle, should be from 0-180    
  } 
  digitalWrite(13, LOW-digitalRead(13));  //toggle led
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter_pub);
  nh.advertise(chatter_pub1);
  nh.advertise(pot_pub);
  nh.advertise(pot_proc);
  nh.advertise(fsr_pub);
  nh.advertise(fsr_proc);

  //pot_array_raw.layout.dim_length = 1;
  pot_array_raw.data_length = 9; 
  //pot_array.layout.dim_length = 1;
  pot_array.data_length = 9;
  //fsr_array_raw.layout.dim_length = 1;
  fsr_array_raw.data_length = 5;
  //fsr_array.layout.dim_length = 1;
  fsr_array.data_length = 5;

  servo[0].attach(2);
  servo[1].attach(3);
  servo[2].attach(4);
  servo[3].attach(5);
  servo[4].attach(6);
  servo[5].attach(7);
  //servo.attach(9); //attach it to pin 9
}

void loop(){

  int temp = 0;
  
	for(int i = 0; i < 9; i++){
      //temp = rand() % 90;
      //Serial.print(temp);
		pot_array_raw.data[i] = analogRead(i);
    //msg_pub1.data = temp;
    //chatter_pub1.publish(&msg_pub1);		
		pot_array.data[i] = pot_array_raw.data[i]*333.0/1024.0; //Conversion function detailed in Datasheet
		}
   pot_pub.publish(&pot_array_raw);
   pot_proc.publish(&pot_array);

	float fsr_voltage = 0;
  	for(int i = 0; i < 5; i++){
    	temp = analogRead(FSR1+i);
      fsr_array_raw.data[i] = temp;
    	//Convert raw values to forces in grams
    	fsr_voltage = temp*5.0/1024.0;  //Convert raw value to voltage
    	fsr_array.data[i] = fsrVolt2Grams(fsr_voltage);  //Convert voltage to Force		
		}
   fsr_pub.publish(&fsr_array_raw);
   fsr_proc.publish(&fsr_array);		
  nh.spinOnce();
  delay(500);
}
//A function which takes a voltage from an FSR and converts it into the force in grams
//NOTE: Only works in 1-3.5V range outside this range it will read either 0 or 1128 (the min and max values)
int fsrVolt2Grams(float voltage){
  int grams;
  if(voltage <= 1.0) grams = 0;
  else if(voltage <= 3.5) grams = (53.2*pow(voltage,4))-(379.2*pow(voltage,3))+(1091.6*pow(voltage,2))-(1288.6*voltage)+541.2; //Conversion function derivation found in companion Excel Sheet
  else grams = 1128;
  return grams;
}

//We average the analog reading to eliminate some of the noise
int averageAnalog(int pin){
 int v=0;
      for(int i=0; i<4; i++) v+= analogRead(pin);
      return v/4;
}
