/*********************************************************************
  Set the servo to 90 degrees

  @Author: YFROBOT-WST
  @Version: V1
  @Date: 07/13/2023
  @URL: www.yfrobot.com.cn
  @par :Copyright © 2022-2023, JianSu YFROBOT

  *Please use this code in accordance with the file license
**********************************************************************/
#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;

#define SERVOPIN1 8   //Servo0  Base
#define SERVOPIN2 9   //Servo1    |
#define SERVOPIN3 10  //Servo2    |
#define SERVOPIN4 11  //Servo3   \|/
#define SERVOPIN5 12  //Servo4  Claw rotation
#define SERVOPIN6 13  //Servo5  Hand nails

void setup() {
  myservo1.attach(SERVOPIN1);
  myservo2.attach(SERVOPIN2);
  myservo3.attach(SERVOPIN3);
  myservo4.attach(SERVOPIN4);
  myservo5.attach(SERVOPIN5);
  myservo6.attach(SERVOPIN6);

  myservo1.write(0);
  myservo2.write(0);
  myservo3.write(0);
  myservo4.write(0);
  myservo5.write(0);
  myservo6.write(0);
  delay(1000);
  myservo1.write(180);
  myservo2.write(180);
  myservo3.write(180);
  myservo4.write(180);
  myservo5.write(180);
  myservo6.write(180);
  delay(1000);
}

void loop() {
  myservo1.write(90);
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
  myservo5.write(90);
  myservo6.write(90);
  delay(1000);
}
