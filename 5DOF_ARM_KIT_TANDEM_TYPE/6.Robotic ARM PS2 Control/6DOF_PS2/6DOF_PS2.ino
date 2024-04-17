/*********************************************************************
  6DOF-PS2

  @Author: YFROBOT-WST
  @Version: V1
  @Date: 05/29/2023
  @URL: www.yfrobot.com.cn
  @par :Copyright © 2022-2032, JianSu YFROBOT
**********************************************************************/
#include <Servo.h>
#include "PS2X_lib.h"

PS2X ps2x;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;

/******************************************
   Set PS2 Pin
   设置PS2手柄接线端口，默认设置
 *****************************************/
#define PS2_DAT 10
#define PS2_CMD 11  //PS2 pin
#define PS2_SEL 12
#define PS2_CLK 13

#define SERVOPIN1 4  //Servo0  转盘
#define SERVOPIN2 5  //Servo1  抬臂1
#define SERVOPIN3 6  //Servo2  抬臂2
#define SERVOPIN4 7  //Servo3  抬臂3
#define SERVOPIN5 8  //Servo4  旋转
#define SERVOPIN6 9  //Servo5  手爪

//IA 初始角度
int SERVO1MID = 90;  // 底部旋转初始角度
int SERVO2MID = 90;  // 底部抬臂初始角度
int SERVO3MID = 90;  // 中间抬臂初始角度
int SERVO4MID = 90;  // 头部抬臂初始角度
int SERVO5MID = 90;  // 手爪旋转初始角度
int SERVO6MID = 90;  // 手爪张开角度初始角度

//抓取目标角度值   水平抓取角度
int SERVO1MIN = 0;       // 底部旋转目标最小角度
int SERVO1MAX = 180;     // 底部旋转目标最大角度
int SERVO2Target = 50;   // 底部抬臂目标角度
int SERVO3Target = 120;  // 中间抬臂目标角度
int SERVO4Target = 40;   // 头部抬臂目标角度
int SERVO5MIN = 0;       // 手爪旋转最小角度
int SERVO5MAX = 180;     // 手爪旋转最大角度
int SERVO6Target = 140;  // 手爪闭合角度

int SERVOPOS1 = SERVO1MID;
int SERVOPOS2 = SERVO2MID;
int SERVOPOS3 = SERVO3MID;
int SERVOPOS4 = SERVO4MID;
int SERVOPOS5 = SERVO5MID;
int SERVOPOS6 = SERVO6MID;

#define pressures false  //Key analog value
#define rumble true      //Vibration motor
int error = 0;
byte type = 0;
byte vibrate = 0;
void (*resetFunc)(void) = 0;  //reset

void setup() {
  Serial.begin(115200);  // Baud rate
  myservo1.attach(SERVOPIN1);
  myservo2.attach(SERVOPIN2);
  myservo3.attach(SERVOPIN3);
  myservo4.attach(SERVOPIN4);
  myservo5.attach(SERVOPIN5);
  myservo6.attach(SERVOPIN6);
  myservo1.write(SERVO1MID);
  myservo2.write(SERVO2MID);
  myservo3.write(SERVO3MID);
  myservo4.write(SERVO4MID);
  myservo5.write(SERVO5MID);
  myservo6.write(SERVO6MID);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  delay(500);
}

void loop() {
  if (error == 1)  //skip loop if no controller found
    resetFunc();
  ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed

  if (ps2x.Button(PSB_START))  //will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  /************************************************************
    手臂按键控制部分，前后左右按键控制，向前水平抓取.
   ***********************************************************/
  if (ps2x.Button(PSB_TRIANGLE)) {  //按键三角形
    SERVOPOS2 -= 2;
    SERVOPOS3 += 1;
    SERVOPOS4 -= 2;
    if (SERVOPOS2 < SERVO2Target) {
      SERVOPOS2 = SERVO2Target;
    }
    if (SERVOPOS3 > SERVO3Target) {
      SERVOPOS3 = SERVO3Target;
      vibrate = 100;
    }
    if (SERVOPOS4 < SERVO4Target) {
      SERVOPOS4 = SERVO4Target;
    }
    myservo2.write(SERVOPOS2);
    myservo3.write(SERVOPOS3);
    myservo4.write(SERVOPOS4);
  }

  if (ps2x.Button(PSB_CROSS)) {  //按键叉
    SERVOPOS2 += 2;
    SERVOPOS3 -= 1;
    SERVOPOS4 += 2;
    if (SERVOPOS2 > SERVO2MID) {
      SERVOPOS2 = SERVO2MID;
    }
    if (SERVOPOS3 < SERVO3MID) {
      SERVOPOS3 = SERVO3MID;
      vibrate = 100;
    }
    if (SERVOPOS4 > SERVO4MID) {
      SERVOPOS4 = SERVO4MID;
    }
    myservo2.write(SERVOPOS2);
    myservo3.write(SERVOPOS3);
    myservo4.write(SERVOPOS4);
  }

  if (ps2x.Button(PSB_CIRCLE)) {  // 圆圈右转
    SERVOPOS1 += 2;
    if (SERVOPOS1 > SERVO1MAX) {
      SERVOPOS1 = SERVO1MAX;
      vibrate = 200;
    }
    myservo1.write(SERVOPOS1);
  }

  if (ps2x.Button(PSB_SQUARE)) {  // 正方形左转
    SERVOPOS1 -= 2;
    if (SERVOPOS1 < SERVO1MIN) {
      SERVOPOS1 = SERVO1MIN;
      vibrate = 200;
    }
    myservo1.write(SERVOPOS1);
  }

  if (ps2x.Button(PSB_R1)) {  // 手爪旋转
    SERVOPOS5 += 2;
    if (SERVOPOS5 > SERVO5MAX) {
      SERVOPOS5 = SERVO5MAX;
      vibrate = 200;
    }
    myservo5.write(SERVOPOS5);
  }

  if (ps2x.Button(PSB_R2)) {  // 手爪旋转
    SERVOPOS5 -= 2;
    if (SERVOPOS5 < SERVO5MIN) {
      SERVOPOS5 = SERVO5MIN;
      vibrate = 200;
    }
    myservo5.write(SERVOPOS5);
  }

  if (ps2x.ButtonPressed(PSB_L1)) {  // 手爪夹紧
    SERVOPOS6 += 3;
    if (SERVOPOS6 > SERVO6Target) {
      SERVOPOS6 = SERVO6Target;
      vibrate = 200;
    }
    myservo6.write(SERVOPOS6);
  }

  if (ps2x.ButtonPressed(PSB_L2)) {  // 手爪松开
    SERVOPOS6 -= 3;
    if (SERVOPOS6 < SERVO6MID) {
      SERVOPOS6 = SERVO6MID;
      vibrate = 200;
    }
    myservo6.write(SERVOPOS6);
  }
  if (ps2x.ButtonPressed(PSB_R1)) {  // 手爪夹紧
    SERVOPOS4 += 4;
    if (SERVOPOS4 > SERVOPAWTA1) {
      SERVOPOS4 = SERVOPAWTA1;
      if (SERVOPOS4 = SERVOPAWTA1) {
        vibrate = 200;
      }
    }
    myservo4.write(SERVOPOS4);
    Serial.print("SERVO4:");
    Serial.println(SERVOPOS4);
  }

  if (ps2x.ButtonPressed(PSB_R2)) {  // 手爪松开
    SERVOPOS4 -= 4;
    if (SERVOPOS4 < SERVOPAWIA) {
      SERVOPOS4 = SERVOPAWIA;
      if (SERVOPOS4 = SERVOPAWIA) {
        vibrate = 200;
      }
    }
    myservo4.write(SERVOPOS4);
    Serial.print("SERVO4:");
    Serial.println(SERVOPOS4);
  }

  if (ps2x.ButtonPressed(PSB_L3)) {
    MAXSPEEDA -= 5;
    if (MAXSPEEDA < 0) {
      MAXSPEEDA = 0;
    }
  }

  if (ps2x.ButtonPressed(PSB_R3)) {
    MAXSPEEDB -= 5;
    if (MAXSPEEDB <= 0) {
      MAXSPEEDB = 0;
    }
  }

  if (ps2x.ButtonPressed(PSB_SELECT)) {
  }

  if (ps2x.ButtonPressed(PSB_PAD_UP)) {  //按键上按下
  }

  if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {  //按键下按下
  }

  if (ps2x.Button(PSB_PAD_RIGHT)) {  //按键右状态变化
  }
  if (ps2x.Button(PSB_PAD_LEFT)) {  //按键左状态变化
  }
  if (ps2x.ButtonPressed(PSB_L3)) {  //按键L3按下
  }
  if (ps2x.ButtonPressed(PSB_R3)) {  //按键R3按下
  }
  // 按键松开，停止震动
  if (ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonReleased(PSB_L2) || ps2x.ButtonReleased(PSB_R2) || ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CIRCLE) || ps2x.ButtonReleased(PSB_CROSS) || ps2x.ButtonReleased(PSB_SELECT)) {
    vibrate = 0;
  }

  Serial.print("SERVO1:");
  Serial.print(SERVOPOS1);
  Serial.print("  2:");
  Serial.print(SERVOPOS2);
  Serial.print("  3:");
  Serial.print(SERVOPOS3);
  Serial.print("  4:");
  Serial.print(SERVOPOS4);
  Serial.print("  5:");
  Serial.print(SERVOPOS5);
  Serial.print("  6:");
  Serial.println(SERVOPOS6);
  delay(50);
}
