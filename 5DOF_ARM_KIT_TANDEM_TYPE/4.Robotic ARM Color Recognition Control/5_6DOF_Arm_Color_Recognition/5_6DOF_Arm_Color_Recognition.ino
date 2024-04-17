/********************************************************************************************************
  5/6自由度机械手逆运动学控制示例-颜色识别抓取控制

  通过颜色传感器识别RGB值控制机械臂颜色分类，。

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
#include <Wire.h>
#include <Servo.h>  // 舵机库文件
#include <Adafruit_TCS34725.h>

// #define DEBUG  // 去掉注释串口打印

int time = 10;  // 递增递减时间
int val = 1;    // 递增递减值

int pin_servo1 = 8;    // 底部旋转舵机（黄色电缆）连接到arduino板上的引脚8
int pin_servo2 = 9;    // 抬臂1舵机（黄色电缆）连接到arduino板上的引脚9
int pin_servo3 = 10;   // 抬臂2舵机（黄色电缆）连接到arduino板上的引脚10
int pin_servo4 = 11;   // 抬臂3舵机（黄色电缆）连接到arduino板上的引脚11
int pin_servo5 = 12;   // 手爪旋转舵机（黄色电缆）连接到arduino板上的引脚12
int pin_gripper = 13;  // 手爪舵机（黄色电缆）连接到arduino板上的引脚13

float Pi = 3.14;  // π取值
// float L0 = 60 + 30;  // 30为机械臂底部圆盘距离检测边缘距离，根据实际调整,60为圆盘底座固定值。
float L1 = 72;   // 抓取物体表面到第二关节位置高度  unit:mm     72为默认高度
float L2 = 105;  // 第2个关节到第3个关节长度
float L3 = 128;  // 第3关节到第4关节的长度  145
float L4 = 180;  // 第4个关节到手臂尖端的长度（夹具）,包含手爪旋转舵机，5自由度为135

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

byte gammatable[256];  // our RGB -> eye-recognized gamma color

//MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

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
  Serial.begin(115200);
  servo1.attach(pin_servo1, min_PWM = 500.0, max_PWM = 2500.00);  // 设定舵机运行范围500-2500对应0-180度
  servo2.attach(pin_servo2, min_PWM = 500.0, max_PWM = 2350.00);
  servo3.attach(pin_servo3, min_PWM = 500.0, max_PWM = 2470.00);
  servo4.attach(pin_servo4, min_PWM = 690.0, max_PWM = 2300.00);  // 修改参数，微调舵机误差
  servo5.attach(pin_servo5, min_PWM = 550.0, max_PWM = 2500.00);
  gripper.attach(pin_gripper);  //手爪

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;  // halt!
  }

  //感谢 PhilB 提供这个伽玛表！它有助于将 RGB 颜色转换为人类看到的颜色
  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    gammatable[i] = x;
  }
}

void loop() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(false);  // turn off LED

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red;
  r /= sum;

  g = green;
  g /= sum;

  b = blue;
  b /= sum;

  r *= 256;
  g *= 256;
  b *= 256;

  if (r > 100 && r < 110 && g > 65 && g < 70 && b > 80 && b < 90) {  // 识别红色运行
    Crawl();                                                         // 抓取动作

    for (int x = 0; x <= 250; x += val) {
      Inverse_kinematics(x, 150, 200);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    for (int z = 200; z >= 130; z -= val) {
      Inverse_kinematics(250, 150, z);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    gripper.write(90);
    delay(500);

    for (int z = 130; z <= 200; z += val) {
      Inverse_kinematics(250, 150, z);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    for (int x = 250; x >= 0; x -= val) {
      Inverse_kinematics(x, 150, 200);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    for (int y = 150; y <= 200; y += val) {
      Inverse_kinematics(0, y, 200);  // 参数给到逆运动学公式
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);
  } else if (r > 48 && r < 55 && g > 130 && g < 140 && b > 55 && b < 65) {
    Crawl();

    for (int x = 0; x >= -250; x -= val) {
      Inverse_kinematics(x, 150, 200);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    for (int z = 200; z >= 130; z -= val) {
      Inverse_kinematics(-250, 150, z);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    gripper.write(90);
    delay(500);

    for (int z = 130; z <= 200; z += val) {
      Inverse_kinematics(-250, 150, z);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    for (int x = -250; x <= 0; x += val) {
      Inverse_kinematics(x, 150, 200);  // 参数给到逆运动学公式
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);

    for (int y = 150; y <= 200; y += val) {
      Inverse_kinematics(0, y, 200);  // 参数给到逆运动学公式
      servo2.write(Theta_2);
      servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
      delay(time);
    }
    delay(500);
  } else {
    Inverse_kinematics(0, 200, 200);  // 参数给到逆运动学公式
    servo1.write(Theta_1);
    servo2.write(Theta_2);
    servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    servo5.write(90);
    gripper.write(90);
    delay(500);
  }

#ifdef DEBUG
  Serial.print("\t");
  // Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  // Serial.print((int)r, BIN); Serial.print((int)g, BIN); Serial.print((int)b, BIN);
  Serial.print((int)r);
  Serial.print("\t");
  Serial.print((int)g);
  Serial.print("\t");
  Serial.print((int)b);
  Serial.println();
#endif
}

void Crawl() {  // 抓取动作
  for (int y = 200; y <= 280; y += val) {
    Inverse_kinematics(0, y, 200);  // 参数给到逆运动学公式
    servo2.write(Theta_2);
    servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    delay(time);
  }
  delay(500);

  for (int z = 200; z >= 35; z -= val) {
    Inverse_kinematics(0, 280, z);  // 参数给到逆运动学公式
    servo2.write(Theta_2);
    servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    delay(time);
  }
  delay(500);

  gripper.write(130);
  delay(500);

  for (int z = 35; z <= 200; z += val) {
    Inverse_kinematics(0, 280, z);  // 参数给到逆运动学公式
    servo2.write(Theta_2);
    servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    delay(time);
  }
  delay(500);

  for (int y = 280; y >= 150; y -= val) {
    Inverse_kinematics(0, y, 200);  // 参数给到逆运动学公式
    servo2.write(Theta_2);
    servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
    delay(time);
  }
  delay(500);
}