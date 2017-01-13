#include <Servo.h>
#define NUM_SERVOS 6  // number of servos that exist in the hand
#define SERVO_MIN           // minimum angle the servo can reach 
#define SERVO_MAX           // maximum angle the servo can reach

Servo servo_test[NUM_SERVOS];
int angle[NUM_SERVOS] = {0};   //angle from beaglebone
//int angle1 = 0;
//int angle2 = 0;
//int angle3 = 0;
//int angle4 = 0;
//int angle5 = 0;
//int angle6 = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
// use this for testing servos with Arduino Mega  
  servo_test[0].attach(3);
  servo_test[1].attach(4);
  servo_test[2].attach(14);
  servo_test[3].attach(15);
  servo_test[4].attach(16);
  servo_test[5].attach(17);

// use this if the pin assignments are consecutive
//  for(int i=0; i < NUM_SERVOS; i++){          
//    servo_test[i].attach(FIRST_SERVO_PIN+i);
//  }
}

// command to rotate servo to specified angle. No "soft rotate"
void loop(){
  for (int i=0;i < NUM_SERVOS; i++){
    servo_test[i].write(angle[i]);
  }
}
