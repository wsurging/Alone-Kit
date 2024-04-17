/********************************************************************************************************
  6自由度机械手逆运动学控制示例1-水平抓取
  手爪和旋转手爪部位可通过自行设定角度，机械臂可通过识别给定角度位置

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
  @Date: 07/03/2023
  @URL: www.yfrobot.com.cn
********************************************************************************************************/
#include <Servo.h>
#include "HUSKYLENS.h"

// #define DEBUG  // Uncomment to turn on debugging output

HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL
void printResult(HUSKYLENSResult result);

Servo servo1;   // 旋转伺服
Servo servo2;   // 第二伺服
Servo servo3;   // 第三伺服
Servo servo4;   // 第四伺服
Servo servo5;   // 第五伺服
Servo gripper;  // 手爪伺服

int pin_servo1 = 8;    // 底部旋转舵机（黄色电缆）连接到arduino板上的引脚8
int pin_servo2 = 9;    // 抬臂1舵机（黄色电缆）连接到arduino板上的引脚9
int pin_servo3 = 10;   // 抬臂2舵机（黄色电缆）连接到arduino板上的引脚10
int pin_servo4 = 11;   // 抬臂3舵机（黄色电缆）连接到arduino板上的引脚11
int pin_servo5 = 12;   // 手爪旋转舵机（黄色电缆）连接到arduino板上的引脚12
int pin_gripper = 13;  // 手爪舵机（黄色电缆）连接到arduino板上的引脚13

float Pi = 3.14;  // π取值
float L1 = 72;    // 抓取物体表面到第二关节位置高度  unit:mm
float L2 = 105;   // 第2个关节到第3个关节长度
float L3 = 128;   // 第3关节到第4关节的长度  145
float L4 = 175;   // 第4个关节到手臂尖端的长度（夹具）,包含手爪旋转舵机

float X_EE;  // 手爪的x轴坐标-左右旋转
float Y_EE;  // 手爪的y轴坐标-前后伸展
float Z_EE;  // 手爪的z轴坐标-高度

float Zoffset, D, d, R;
float alpha1, alpha2, alpha3;
float Theta_1, Theta_2, Theta_3, Theta_4;

float min_PWM;  //default arduino 500
float max_PWM;  //default arduino 2500
// theta_4角度经过调整，实际是180+90-alpha3，舵机精度问题做调整。
void Inverse_kinematics(double X_EE, double Y_EE, double Z_EE) {  //机械臂逆运动学，给定X,Y,Z坐标
  if (X_EE > 0 && Z_EE >= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
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
    D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
    Theta_1 = (atan(Y_EE / X_EE)) * (180.00 / Pi);  //theta 1
    d = D - L4;
    Zoffset = Z_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha2 - alpha1);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha 3
    Theta_4 = 180 + 73 - alpha3;                                                                //theta 4
  } else if (X_EE == 0 && Z_EE >= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
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
    D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
    Theta_1 = 90.00;  //theta 1
    d = D - L4;
    Zoffset = Z_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha2 - alpha1);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha 3
    Theta_4 = 180 + 73 - alpha3;                                                                //theta 4
  } else if (X_EE < 0 && Z_EE >= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
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
    D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
    Theta_1 = 90.00 + (90.00 - abs((atan(Y_EE / X_EE)) * (180.00 / Pi)));  //theta 1
    d = D - L4;
    Zoffset = Z_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha2 - alpha1);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - alpha2 - Theta_3) + alpha1);                                   //alpha 3
    Theta_4 = 180 + 73 - alpha3;                                                                //theta 4
  }
}

void setup() {
  Serial.begin(115200);
  servo1.attach(pin_servo1, min_PWM = 500.0, max_PWM = 2500.00);
  servo2.attach(pin_servo2, min_PWM = 500.0, max_PWM = 2500.00);
  servo3.attach(pin_servo3, min_PWM = 500.0, max_PWM = 2500.00);
  servo4.attach(pin_servo4, min_PWM = 500.0, max_PWM = 2500.00);
  servo5.attach(pin_servo5, min_PWM = 500.0, max_PWM = 2500.00);
  gripper.attach(pin_gripper);  //gripper
  Wire.begin();
  while (!huskylens.begin(Wire)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);  //将算法切换为对象跟踪
}

void loop() {
  if (!huskylens.request()) {
    Inverse_kinematics(0, 200, 200);  //手爪坐标
#ifdef DEBUG
    Serial.print(F("Fail, recheck the connection!"));
#endif
  } else if (!huskylens.isLearned()) {
    Inverse_kinematics(0, 200, 200);  //手爪坐标
#ifdef DEBUG
    Serial.print(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
#endif
  } else if (!huskylens.available()) {
    Inverse_kinematics(0, 200, 200);  //手爪坐标
#ifdef DEBUG
    Serial.print(F("No block or arrow appears on the screen!"));
#endif
  } else {
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);
      //通过AI识别物体的位置，转化为X,Y坐标，Z左边根据物体高度直接给定数值。
      int X = map(result.xCenter, 0, 320, 165, -135);
      int Y = map(result.yCenter, 0, 240, 240, 0);
#ifdef DEBUG
      Serial.print(" X: ");
      Serial.print(X);
      Serial.print(" Y: ");
      Serial.print(Y);
#endif
      //运行一组抓取物体的动作，控制坐标变化使舵机平稳运行
      if (X >= 0) {
        for (int x = 0; x <= X; x += 1) {
          Inverse_kinematics(x, 200, 200);  // Inverse_kinematics(左右，前后，高度)  单位mm
          servo1.write(Theta_1);
          servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
          servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
          servo2.write(Theta_2);
          delay(20);
        }
      } else {
        for (int x = 0; x >= X; x -= 1) {
          Inverse_kinematics(x, 200, 200);  // Inverse_kinematics(左右，前后，高度)  单位mm
          servo1.write(Theta_1);
          servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
          servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
          servo2.write(Theta_2);
          delay(20);
        }
      }

      for (int y = 20; y <= Y; y += 1) {
        Inverse_kinematics(X, (180 + y), 200);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo1.write(Theta_1);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo2.write(Theta_2);
        delay(20);
      }

      for (int z = 200; z >= 30; z -= 2) {
        Inverse_kinematics(X, (180 + Y), z);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo1.write(Theta_1);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo2.write(Theta_2);
        delay(20);
      }

      gripper.write(130);  //手爪夹紧
      delay(500);

      for (int z = 30; z <= 200; z += 2) {
        Inverse_kinematics(X, (180 + Y), z);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo2.write(Theta_2);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo1.write(Theta_1);
        servo5.write(90);
        gripper.write(130);
        delay(20);
      }

      for (int y = Y; y >= 0; y -= 2) {
        Inverse_kinematics(X, (180 + y), 200);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo1.write(Theta_1);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo2.write(Theta_2);
        delay(20);
      }

      for (int x = X; x <= 270; x += 2) {
        Inverse_kinematics(x, 180, 200);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo1.write(Theta_1);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo2.write(Theta_2);
        servo5.write(90);
        gripper.write(130);
        delay(20);
      }

      for (int z = 200; z >= 130; z -= 2) {
        Inverse_kinematics(270, 180, z);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo2.write(Theta_2);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo1.write(Theta_1);
        servo5.write(90);
        gripper.write(130);
        delay(20);
      }

      gripper.write(90);  //手爪松开
      delay(500);

      for (int z = 130; z <= 200; z += 2) {
        Inverse_kinematics(270, 180, z);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo2.write(Theta_2);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo1.write(Theta_1);
        servo5.write(90);
        gripper.write(90);
        delay(20);
      }
      for (int x = 270; x >= 0; x -= 2) {
        Inverse_kinematics(x, 200, 200);  // Inverse_kinematics(左右，前后，高度)  单位mm
        servo1.write(Theta_1);
        servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
        servo2.write(Theta_2);
        servo5.write(90);
        gripper.write(90);
        delay(20);
      }
      // delay(500);
    }
  }
  servo1.write(Theta_1);
  servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
  servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
  servo2.write(Theta_2);
  servo5.write(90);
  gripper.write(90);
  delay(100);

#ifdef DEBUG
  Serial.print(" Servo: ");
  Serial.print(Theta_1);
  Serial.print("     ");
  Serial.print(Theta_2);
  Serial.print("     ");
  Serial.print(180 - Theta_3);
  Serial.print("     ");
  Serial.println(180 - Theta_4);
#endif
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
#ifdef DEBUG
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
#endif
  } else if (result.command == COMMAND_RETURN_ARROW) {
#ifdef DEBUG
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
#endif
  } else {
#ifdef DEBUG
    Serial.println("Object unknown!");
#endif
  }
}
