/********************************************************************************************************
  5/6自由度机械手逆运动学控制示例-电位器控制

  通过4/5个电位器控制机械臂，3个控制X,Y,Z轴，其余2个控制手爪旋转和抓取，水平演示可以不使用旋转舵机和电位器。
  电位器初始状态都调节到中间位置，不需要很精确，手爪调节到最小值。

  注意：算法没有限制角度范围，位置达不到的状态运行不正常。

                              (5/6自由度增加或减少此关节)
        S3  0°        S4  0°     S5（旋转）        
      90° |---------------|-------|-------  90°
          | 180°          180°            S6 （手爪）       
          |
          |
          |
          | 90°
   180°   | S2     0°
   -----------------
          S1（底座旋转）
  
  舵机每个都调整好角度后再安装，设置角度为90度（中间角度），舵机分布安装上图；
  舵机S3,S4如果角度上为180°，下为0°则底部代码需将代码(180-theta3)改为theta3，theta4同理。
  S1，S2如安装角度方向相反也同样需要使用补角，改为(180-theta1)，(180-theta2)。

  @Author: YFROBOT-WST
  @Version: V1.0
  @Date: 07/13/2023
  @URL: www.yfrobot.com.cn
  @par :Copyright © 2022-2023, JianSu YFROBOT
********************************************************************************************************/
#include <Servo.h>  // 舵机库文件

// #define DEBUG  // 去掉注释串口打印

#define XPin A0   // 电位器连接端口
#define YPin A1   // 电位器连接端口
#define ZPin A2   // 电位器连接端口
#define S5Pin A3  // 电位器连接端口
#define GPin A4   // 电位器连接端口

int pin_servo1 = 8;    // 底部旋转舵机（黄色电缆）连接到arduino板上的引脚8
int pin_servo2 = 9;    // 抬臂1舵机（黄色电缆）连接到arduino板上的引脚9
int pin_servo3 = 10;   // 抬臂2舵机（黄色电缆）连接到arduino板上的引脚10
int pin_servo4 = 11;   // 抬臂3舵机（黄色电缆）连接到arduino板上的引脚11
int pin_servo5 = 12;   // 手爪旋转舵机（黄色电缆）连接到arduino板上的引脚12
int pin_gripper = 13;  // 手爪舵机（黄色电缆）连接到arduino板上的引脚13

float Pi = 3.14;     // π取值
float L0 = 60 + 30;  // 30为机械臂底部圆盘距离检测边缘距离，根据实际调整,60为圆盘底座固定值。
float L1 = 90;       // 抓取物体表面到第二关节位置高度  unit:mm     72为默认高度
float L2 = 105;      // 第2个关节到第3个关节长度
float L3 = 128;      // 第3关节到第4关节的长度  145
float L4 = 135;      // 第4个关节到手臂尖端的长度（夹具）,包含手爪旋转舵机175

float X_EE, Y_EE, Z_EE;  // 手爪的x轴坐标-左右旋转,y轴坐标-前后伸展,z轴坐标-高度
float Zoffset, D, d, R;
float alpha1, alpha2, alpha3;
float Theta_1, Theta_2, Theta_3, Theta_4;

float min_PWM;  //default arduino 500
float max_PWM;  //default arduino 2500

Servo servo1;   // 旋转伺服
Servo servo2;   // 第二伺服
Servo servo3;   // 第三伺服
Servo servo4;   // 第四伺服
Servo servo5;   // 第五伺服
Servo gripper;  // 手爪伺服

int XVal, YVal, ZVal, S5Val, GVal;

void Inverse_kinematics(double X_EE, double Y_EE, double Z_EE) {  // 机械臂逆运动学，给定X(左右),Y（前后）,Z（高度）坐标
  D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
  if (D > L4 + 60 || D < L4 + 60 && Z_EE > (L1 + L2 - L4)) {
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
  }
}

void setup() {
  Serial.begin(115200);                                           // 波特率
  servo1.attach(pin_servo1, min_PWM = 500.0, max_PWM = 2500.00);  // 设定舵机运行范围500-2500对应0-180度
  servo2.attach(pin_servo2, min_PWM = 500.0, max_PWM = 2350.00);
  servo3.attach(pin_servo3, min_PWM = 500.0, max_PWM = 2470.00);
  servo4.attach(pin_servo4, min_PWM = 780.0, max_PWM = 2310.00);  // 修改参数，微调舵机误差
  servo5.attach(pin_servo5, min_PWM = 500.0, max_PWM = 2500.00);
  gripper.attach(pin_gripper);  //手爪
}

void loop() {
  XVal = analogRead(XPin);
  YVal = analogRead(YPin);
  ZVal = analogRead(ZPin);
  S5Val = analogRead(S5Pin);
  GVal = analogRead(GPin);

  int Xdis = map(XVal, 0, 1023, 260, -260);         // X轴左右旋转
  int Ydis = map(YVal, 0, 1023, 0, L1 + L2 + L3);   // Y轴前后距离
  int Zdis = map(ZVal, 0, 1023, 20, L1 + L2 + L3);  // Z轴高度
  int S5 = map(S5Val, 0, 1023, 0, 180);             // 手爪旋转角度，默认0-180度范围，水平抓取时没什么用
  int Gripper = map(GVal, 0, 1023, 90, 130);        // 手爪张开最大时为90度，闭合130，适当调整以适配

  Inverse_kinematics(Xdis, Ydis, Zdis);  // 通过电位器值给到逆运动学公式中
  servo1.write(Theta_1);
  servo2.write(Theta_2);
  servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
  servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
  servo5.write(90);
  gripper.write(Gripper);
  delay(50);

#ifdef DEBUG
  Serial.print(" XVal: ");
  Serial.print(XVal);
  Serial.print("  YVal: ");
  Serial.print(YVal);
  Serial.print("  ZVal: ");
  Serial.print(ZVal);
  Serial.print("     Xdis: ");
  Serial.print(Xdis);
  Serial.print("  Ydis: ");
  Serial.print(Ydis);
  Serial.print("  Zdis: ");
  Serial.println(Zdis);
#endif
}