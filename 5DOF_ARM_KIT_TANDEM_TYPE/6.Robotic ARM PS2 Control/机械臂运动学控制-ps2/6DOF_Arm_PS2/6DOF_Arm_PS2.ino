/*********************************************************************
  6DOF Arm PS2 Control Example

  控制说明：
          1.上下按键控制机械臂前后伸展，收缩。
          2.左右机械臂旋转。
          3.L1，L2控制机械臂上升、下降。
          4.三角形和叉控制机械臂手爪张开、闭合。
          5.正方形和圆形控制机械臂手爪旋转。
          6.SELECT控制机械臂恢复设置和位置。
          7.L3,R3按键控制运行速度增大、减少。

  @Author: YFROBOT-WST
  @Version: V1.0
  @Date: 11/30/2023
  @URL: www.yfrobot.com.cn
  @par :Copyright © 2022-2032, JianSu YFROBOT
  
  修改说明：无
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
   设置PS2手柄接线端口
 *****************************************/
#define PS2_DAT A0
#define PS2_CMD A1  //PS2 pin
#define PS2_SEL A2
#define PS2_CLK A3

/******************************************
   Set Motor Pin
   设置电机驱动参数，默认设置，不能更改
 *****************************************/
#define SERVOPIN1 8   //Servo0
#define SERVOPIN2 9   //Servo1  抬臂1
#define SERVOPIN3 10  //Servo2  抬臂2
#define SERVOPIN4 11  //Servo3  抬臂3
#define SERVOPIN5 12  //Servo4  手爪旋转
#define SERVOPIN6 13  //Servo5  手爪

int val = 3;         // 机械臂运行速度
int S5MID = 90;      // 手爪初始角度
int S5MINVAL = -90;  // 手爪旋转最小角度范围
int S5MAXVAL = 90;   // 手爪旋转最大角度范围
int S6MID = 100;     // 手爪初始角度
int S6VAL = 50;      // 手爪闭合角度范围

/* 机械臂关节参数 */
float Pi = 3.14;  // π取值
float LMIN = 30;  // 手爪离地面最小值
float L0 = 75;    // 60为机械臂底部距离抓取最近位置距离
float L1 = 70;    // 抬臂到地面高度  unit:mm
float L2 = 105;   // 第2个关节到第3个关节长度
float L3 = 128;   // 第3关节到第4关节的长度
float L4 = 170;   // 第4个关节到手臂尖端的长度（夹具）

float X_EE, Y_EE, Z_EE;  // 手爪的x轴坐标-左右旋转,y轴坐标-前后伸展,z轴坐标-高度
float Zoffset, D, d, R;
float alpha1, alpha2, alpha3;
float Theta_1, Theta_2, Theta_3, Theta_4;
float x, y, z, w, c;

float min_PWM;  //default arduino 500
float max_PWM;  //default arduino 2500

#define pressures false  //Key analog value
#define rumble true      //Vibration motor
int error = 0;
byte type = 0;
byte vibrate = 0;
void (*resetFunc)(void) = 0;  //reset

/* 机械臂逆运动学，竖直抓取和垂直抓取，给定相应X,Y,Z参数 */
void Inverse_kinematics(double X_EE, double Y_EE, double Z_EE) {  // 机械臂逆运动学，给定X(左右),Y（前后）,Z（高度）坐标
  D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
  if ((D > L4 + L0 && Z_EE >= LMIN) || (D < L4 + L0 && Z_EE >= L1)) {
    if (X_EE > 0 && Z_EE >= L1) {
      Theta_1 = (atan(Y_EE / X_EE)) * (180.00 / Pi);  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha1 + alpha2);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE > 0 && Z_EE <= L1) {
      Theta_1 = (atan(Y_EE / X_EE)) * (180.00 / Pi);  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha2 - alpha1);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE == 0 && Z_EE >= L1) {
      Theta_1 = 90.00;  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = alpha1 + alpha2;                                                                  //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE == 0 && Z_EE <= L1) {
      Theta_1 = 90.00;  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha2 - alpha1);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE < 0 && Z_EE >= L1) {
      Theta_1 = 90.00 + (90.00 - abs((atan(Y_EE / X_EE)) * (180.00 / Pi)));  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha1 + alpha2);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE < 0 && Z_EE <= L1) {
      Theta_1 = 90.00 + (90.00 - abs((atan(Y_EE / X_EE)) * (180.00 / Pi)));  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha2 - alpha1);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - alpha2 - Theta_3) + alpha1);                                   //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    }
  } else if (D <= L0 + L4 && D >= L0 && Z_EE <= L1) {
    if (X_EE > 0 && Z_EE >= 0) {
      Theta_1 = (atan(Y_EE / X_EE)) * (180.00 / Pi);  //theta 1
      Zoffset = Z_EE + L4 - L1;
      R = sqrt(pow(D, 2) + pow(Zoffset, 2));
      alpha1 = (acos(D / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha1 + alpha2);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 360 - 90 - Theta_2 - Theta_3;
      Theta_4 = alpha3 - 90;  // theta4
    } else if (X_EE == 0 && Z_EE >= 0) {
      Theta_1 = 90.00;  //theta 1
      Zoffset = Z_EE + L4 - L1;
      R = sqrt(pow(D, 2) + pow(Zoffset, 2));
      alpha1 = (acos(D / R)) * (180.00 / Pi);                                                     // alpha1
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);      // alpha2
      Theta_2 = alpha1 + alpha2;                                                                  // theta2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  // theta3
      alpha3 = 360 - 90 - Theta_2 - Theta_3;                                                      // alpha3
      Theta_4 = alpha3 - 90;                                                                      // theta4
    } else if (X_EE < 0 && Z_EE >= 0) {
      Theta_1 = 90.00 + (90.00 - abs((atan(Y_EE / X_EE)) * (180.00 / Pi)));  //theta 1
      Zoffset = Z_EE + L4 - L1;
      R = sqrt(pow(D, 2) + pow(Zoffset, 2));
      alpha1 = (acos(D / R)) * (180.00 / Pi);                                                     // alpha1
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);      // alpha2
      Theta_2 = alpha1 + alpha2;                                                                  // theta2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  // theta3
      alpha3 = 360 - 90 - Theta_2 - Theta_3;                                                      // alpha3
      Theta_4 = alpha3 - 90;                                                                      // theta4
    }
  }
}

void setup() {
  Serial.begin(115200);  // Baud rate
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  myservo1.attach(SERVOPIN1, min_PWM = 500.0, max_PWM = 2500.00);  // 设定舵机运行范围500-2500对应0-180度，修改参数，微调舵机误差
  myservo2.attach(SERVOPIN2, min_PWM = 500.0, max_PWM = 2500.00);  // 设定舵机运行范围500-2500对应0-180度，修改参数，微调舵机误差
  myservo3.attach(SERVOPIN3, min_PWM = 500.0, max_PWM = 2500.00);  // 设定舵机运行范围500-2500对应0-180度，修改参数，微调舵机误差
  myservo4.attach(SERVOPIN4, min_PWM = 500.0, max_PWM = 2500.00);  // 设定舵机运行范围500-2500对应0-180度，修改参数，微调舵机误差
  myservo5.attach(SERVOPIN5, min_PWM = 770.0, max_PWM = 2400.00);  // 设定舵机运行范围500-2500对应0-180度，修改参数，微调舵机误差
  myservo6.attach(SERVOPIN6);
  Inverse_kinematics(0, L3 + L4 + y, L1 + L2 + z);
  myservo5.write(S5MID);
  myservo6.write(S6MID);
  delay(500);
}

void loop() {
  if (error == 1)  //skip loop if no controller found
    resetFunc();

  ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed

  if (ps2x.Button(PSB_START))  //will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  /**************************************************************************
    机械臂控制部分
  **************************************************************************/
  if (ps2x.Button(PSB_PAD_UP)) {  // 向前伸展手臂
    y += val;
    Inverse_kinematics(x, L3 + L4 + y, L1 + L2 + z);
  }

  if (ps2x.Button(PSB_PAD_DOWN)) {  // 向后收缩手臂
    y -= val;
    Inverse_kinematics(x, L3 + L4 + y, L1 + L2 + z);
  }

  if (ps2x.Button(PSB_PAD_LEFT)) {  // 向左旋转手臂
    x += val;
    Inverse_kinematics(x, L3 + L4 + y, L1 + L2 + z);
  }

  if (ps2x.Button(PSB_PAD_RIGHT)) {  // 向右旋转手臂
    x -= val;
    Inverse_kinematics(x, L3 + L4 + y, L1 + L2 + z);
  }

  if (ps2x.Button(PSB_L1)) {  // 手臂上升
    z += val;
    Inverse_kinematics(x, L3 + L4 + y, L1 + L2 + z);
  }

  if (ps2x.Button(PSB_L2)) {  // 手臂下降
    z -= val;
    Inverse_kinematics(x, L3 + L4 + y, L1 + L2 + z);
  }

  if (ps2x.Button(PSB_CIRCLE)) {  // 圆圈 手爪旋转
    w += val;
    if (w >= S5MAXVAL) {
      w = S5MAXVAL;
      vibrate = 100;
    }
  }

  if (ps2x.Button(PSB_SQUARE)) {  // 正方形 手爪旋转
    w -= val;
    if (w <= S5MINVAL) {
      w = S5MINVAL;
      vibrate = 100;
    }
  }

  if (ps2x.Button(PSB_TRIANGLE)) {  // 三角形 手爪闭合
    c += 2;
    if (c >= S6VAL) {
      c = S6VAL;
      vibrate = 50;
    }
  }

  if (ps2x.Button(PSB_CROSS)) {  // 叉 手爪松开
    c -= 2;
    if (c <= 0) {
      c = 0;
      vibrate = 50;
    }
  }

  if (ps2x.ButtonPressed(PSB_R1)) {  // 舵机运行速度每次按下增加1
    val += 1;
    if (val >= 10) {
      val = 10;
    }
    vibrate = 50;
  }

  if (ps2x.ButtonPressed(PSB_R2)) {  // 舵机运行速度每次按下减少1
    val -= 1;
    if (val <= 1) {
      val = 1;
    }
    vibrate = 50;
  }

  if (ps2x.ButtonPressed(PSB_SELECT)) {  // 恢复初始设置和状态
    x = 0;
    y = 0;
    z = 0;
    w = 0;
    c = 0;
    Inverse_kinematics(0, L3 + L4, L1 + L2);
    delay(500);
  }

  if (L1 + L2 + z < LMIN) {
    vibrate = 150;
  }

  if (ps2x.ButtonPressed(PSB_L3)) {  // 没有使用

    vibrate = 100;
  }

  if (ps2x.ButtonPressed(PSB_R3)) {  // 没有使用

    vibrate = 100;
  }

  if (Theta_2 >= 0 && Theta_2 <= 180 && Theta_3 >= 0 && Theta_3 <= 180 && Theta_4 >= 0 && Theta_4 <= 180) {
    /* 控制每个舵机运行角度*/
    myservo1.write(180 - Theta_1);  // 舵机安装方向或者旋转方向不同导致角度不正确，选择补角，用180-计算得出的角度
    myservo2.write(180 - Theta_2);  // 舵机安装方向或者旋转方向不同导致角度不正确，选择补角，用180-计算得出的角度
    myservo3.write(180 - Theta_3);  // 舵机安装方向或者旋转方向不同导致角度不正确，选择补角，用180-计算得出的角度
    myservo4.write(180 - Theta_4);  // 舵机安装方向或者旋转方向不同导致角度不正确，选择补角，用180-计算得出的角度
    myservo5.write(S5MID + w);
    myservo6.write(S6MID + c);
  } else {
    vibrate = 150;
  }

  if (ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_L2) || ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonReleased(PSB_R2) || ps2x.ButtonReleased(PSB_PAD_UP) || ps2x.ButtonReleased(PSB_PAD_DOWN) || ps2x.ButtonReleased(PSB_PAD_RIGHT) || ps2x.ButtonReleased(PSB_PAD_LEFT)
      || ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CIRCLE) || ps2x.ButtonReleased(PSB_CROSS) || ps2x.ButtonReleased(PSB_SELECT) || ps2x.ButtonReleased(PSB_R3) || ps2x.ButtonReleased(PSB_L3)) {
    vibrate = 0;
  }

  /***********************************************************************
     摇杆控制部分 Rocker program
  ***********************************************************************/

  if (0 <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= 120) {
  }

  if (0 <= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) <= 120) {
  }

  if (0 <= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) <= 120) {
  }
  Serial.print("X,Y,Z: ");
  Serial.print(x);
  Serial.print(" , ");
  Serial.print(L3 + L4 + y);
  Serial.print(" , ");
  Serial.print(L1 + L2 + z);
  Serial.print("    ");
  Serial.print("SERVO: ");
  Serial.print(180 - Theta_1);
  Serial.print("    ");
  Serial.print(180 - Theta_2);
  Serial.print("    ");
  Serial.print(180 - Theta_3);
  Serial.print("    ");
  Serial.print(180 - Theta_4);
  Serial.print("    ");
  Serial.print(S5MID + w);
  Serial.print("    ");
  Serial.print(S6MID + c);
  Serial.print("    ");
  Serial.print("Speed: ");
  Serial.println(val);
  delay(50);
}
