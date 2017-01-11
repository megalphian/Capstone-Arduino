/*
 Name:    Motor_contrl.ino
 Created: 1/11/2017 2:10:36 PM
 Author:  Megnath
*/

#include <Servo.h>
Servo servo_test[2];
int angle1 = 0;
int angle2 = 90;
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  servo_test[0].attach(14);
  servo_test[1].attach(15);
}

// the loop function runs over and over again until power down or reset
void loop() {
  for(angle1 = 0, angle2 = 90; angle1 < 180||angle2<180; angle1 += 5)    // command to move from 0 degrees to 180 degrees 
  {                                  
    servo_test[0].write(angle1);                 //command to rotate the servo to the specified angle
    if(angle2<180) angle2 +=5;
    servo_test[1].write(angle2);
    delay(5);                       
  } 

 //for(angle2 = 90; angle2 < 180; angle2 += 5)    // command to move from 0 degrees to 180 degrees 
  //{                                  
    //servo_test[1].write(angle2);                 //command to rotate the servo to the specified angle
    //delay(5);                       
  //} 
  delay(1000);

  //servo_test.write(90);
  for(angle1 = 180, angle2=180; angle1>=1||angle2>90; angle1-=5)     // command to move from 180 degrees to 0 degrees 
  {                                
    servo_test[0].write(angle1);              //command to rotate the servo to the specified angle
    if(angle2>90) angle2 -=5;
    servo_test[1].write(angle2); 
    delay(5);                       
  } 

delay(1000);
  //servo_test.write(90);
  //for(angle2 = 180; angle2>=1; angle2-=5)     // command to move from 180 degrees to 0 degrees 
  //{                                
    //servo_test[1].write(angle2);              //command to rotate the servo to the specified angle
    //delay(5); 
//}
}
