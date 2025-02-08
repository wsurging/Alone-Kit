//未测试
#include <Ps3Controller.h>
#include "ESP32_Servo.h"

#define DEBUG  // Uncomment to turn on debugging output

/**********************************************************************
                           PS3无线手柄
**********************************************************************/
int player = 0;
int battery = 0;

/***********************************************************************
                            机械臂配置
***********************************************************************/
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

float servoSpeed = 0.1;  // 机械臂运行速度
int GRISpeed = 1;        // 手爪舵机运行速度

// Arm dimensions(mm) 关节长度尺寸（毫米）
#define BASE_HGT 135  // 底座高度
#define HUMERUS 105   // 肩部到肘部
#define ULNA 128      // 肘部到手腕
#define GRIPPER 130   // 夹具（手腕旋转机构）

// Servo connected to the main control port 舵机连接主控端口
#define BAS 25  // 底座旋转舵机端口号  27
#define SHL 13  // 肩部舵机端口号  26
#define ELB 5   // 肘部舵机端口号 25
#define WRI 23  // 手腕舵机端口号 33
#define GRI 19  // 手爪舵机端口号 32

// Define right angle errors (90 degree value (us))定义直角误差
#define BAS_MID 90  // 底座旋转舵机初始角度
#define SHL_MID 92  // 肩部舵机初始角度
#define ELB_MID 95  // 肘部舵机初始角度
#define WRI_MID 95  // 手腕舵机初始角度
#define GRI_MID 90  // 手爪舵机初始角度
#define GRI_MIN 30  // 手爪舵机最小闭合角度

int g = GRI_MID;

float servo1, servo2, servo3, servo4;
float x, y, z, w;

//float to long conversion 浮点数到长整型的转换
#define ftl(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x)-0.5))

/* pre-calculations 预先计算*/
float hum_sq = HUMERUS * HUMERUS;
float uln_sq = ULNA * ULNA;

void notify() {
  //---------------- Analog stick value events ---------------
  if ((abs(Ps3.data.analog.stick.lx) + abs(Ps3.data.analog.stick.ly) > 2)
      || (abs(Ps3.data.analog.stick.rx) + abs(Ps3.data.analog.stick.ry) > 2)) {  // 满足条件执行摇杆控制
    if (Ps3.data.analog.stick.ly <= -10 && Ps3.data.analog.stick.ly >= -128) {   // 左边Y轴摇杆控制，小于0向前伸展
      y += servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);                     // 给定手爪坐标set_arm(左右(±), 前后(+), 高度(+), 手爪俯仰角度(±))
    } else if (Ps3.data.analog.stick.ly >= 10 && Ps3.data.analog.stick.ly <= 127) {  // 机械臂后移
      y -= servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    } else {  // 摇杆回中机械臂保持最后运动的状态
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    }

    if (Ps3.data.analog.stick.lx <= -10 && Ps3.data.analog.stick.lx >= -128) {  // 左边X轴摇杆控制左右旋转  左侧旋转
      x -= servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    } else if (Ps3.data.analog.stick.lx > 10 && Ps3.data.analog.stick.lx <= 127) {  // 右侧旋转
      x += servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    } else {  // 摇杆回中机械臂保持最后运动的状态
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    }

    if (Ps3.data.analog.stick.ry <= -10 && Ps3.data.analog.stick.ry >= -128) {  // 右边Y轴摇杆控制上下运行 上升
      z += servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    } else if (Ps3.data.analog.stick.ry >= 10 && Ps3.data.analog.stick.ry <= 127) {  // 下降
      z -= servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    } else {  // 摇杆回中机械臂保持最后运动的状态
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    }

    if (Ps3.data.analog.stick.rx < -10 && Ps3.data.analog.stick.rx >= -128) {  // 右边X轴摇杆控制手腕角度 手腕下垂
      w += servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    } else if (Ps3.data.analog.stick.rx > 10 && Ps3.data.analog.stick.rx <= 127) {  // 手腕上扬
      w -= servoSpeed;
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    } else {  // 摇杆回中手腕保持最后运动的状态
      set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    }

#ifdef DEBUG
    Serial.print("ARM-Position:");
    Serial.print("  X:");
    Serial.print(x);
    Serial.print("  Y:");
    Serial.print(ULNA + GRIPPER + y);
    Serial.print("  Z:");
    Serial.print(BASE_HGT + HUMERUS + z);
    Serial.print("    Servo-Angle:");
    Serial.print("  ");
    Serial.print(servo1);
    Serial.print("  ");
    Serial.print(servo2);
    Serial.print("  ");
    Serial.print(servo3);
    Serial.print("  ");
    Serial.print(servo4);
    Serial.print("  W:");
    Serial.print(w);
    Serial.println();
#endif
  }

  if (Ps3.data.button.l1) {  // L1按键控制手爪张开
    g += GRISpeed;
    if (g > GRI_MID) {
      g = GRI_MID;
    }
#ifdef DEBUG
    Serial.print("GRIPPER-Open:");
    Serial.println(g);
#endif
  }

  if (Ps3.data.button.l2) {  // L2按键控制手爪闭合
    g -= GRISpeed;
    if (g < GRI_MIN) {
      g = GRI_MIN;
    }
#ifdef DEBUG
    Serial.print("GRIPPER-Close:");
    Serial.println(g);
#endif
  }

  //--------------- Digital stick button events --------------
  if (Ps3.event.button_down.l3) {  // 按下L3 机械臂运行速度减
    servoSpeed -= 0.1;
    if (servoSpeed < 0.2) {
      servoSpeed = 0.2;
      Ps3.setRumble(50);
    }
#ifdef DEBUG
    Serial.print("servoSpeed:");
    Serial.println(servoSpeed);
#endif
  } else if (Ps3.event.button_up.l3) {  // 松开停止震动
    Ps3.setRumble(0.0);
  }

  if (Ps3.event.button_down.r3) {  // 按下R3 机械臂运行速度增加
    servoSpeed += 0.1;
    if (servoSpeed > 2) {
      servoSpeed = 2;
      Ps3.setRumble(50);
    }
#ifdef DEBUG
    Serial.print("servoSpeed:");
    Serial.println(servoSpeed);
#endif
  } else if (Ps3.event.button_up.r3) {  // 松开停止震动
    Ps3.setRumble(0.0);
  }

  //---------- Digital select/start/ps button events ---------
  if (Ps3.event.button_down.select) {  // SELECT恢复舵机为初始状态 手柄震动提示
    x = 0;
    y = 0;
    z = 0;
    w = 0;
    g = GRI_MID;
    set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    myservo5.write(g);
    Ps3.setRumble(50);
#ifdef DEBUG
    Serial.println("Reset");
#endif
  } else if (Ps3.event.button_up.select) {
    Ps3.setRumble(0.0);
  }

  //---------------------- Battery events ---------------------
  if (Ps3.data.button.start) {  // START按键按下查询手柄电池电量
    if (battery != Ps3.data.status.battery) {
      battery = Ps3.data.status.battery;
      Serial.print("The controller battery is ");
      if (battery == ps3_status_battery_charging) Serial.println("charging");
      else if (battery == ps3_status_battery_full) Serial.println("FULL");
      else if (battery == ps3_status_battery_high) Serial.println("HIGH");
      else if (battery == ps3_status_battery_low) Serial.println("LOW");
      else if (battery == ps3_status_battery_dying) Serial.println("DYING");
      else if (battery == ps3_status_battery_shutdown) Serial.println("SHUTDOWN");
      else Serial.println("UNDEFINED");
    }
  }



  //--- Digital cross/square/triangle/circle button events ---
  // if (Ps3.event.button_down.cross) {
  //   Serial.println("Started pressing the cross button");
  // }

  // if (Ps3.event.button_up.cross) {
  //   Serial.println("Released the cross button");
  // }

  // if (Ps3.event.button_down.square) {
  //   Serial.println("Started pressing the square button");
  // }

  // if (Ps3.event.button_up.square) {
  //   Serial.println("Released the square button");
  // }
  // if (Ps3.event.button_down.triangle) {
  //   Serial.println("Started pressing the triangle button");
  // }

  // if (Ps3.event.button_up.triangle) {
  //   Serial.println("Released the triangle button");
  // }

  // if (Ps3.event.button_down.circle)
  //   Serial.println("Started pressing the circle button");
  // if (Ps3.event.button_up.circle)
  //   Serial.println("Released the circle button");

  // //--------------- Digital D-pad button events --------------
  // if (Ps3.event.button_down.up) {
  //   Serial.println("Started pressing the up button");
  // }

  // if (Ps3.event.button_up.up) {
  //   Serial.println("Released the up button");
  // }

  // if (Ps3.event.button_down.right)
  //   Serial.println("Started pressing the right button");
  // if (Ps3.event.button_up.right)
  //   Serial.println("Released the right button");

  // if (Ps3.event.button_down.down) {
  //   Serial.println("Started pressing the down button");
  // }
  // if (Ps3.event.button_up.down) {
  //   Serial.println("Released the down button");
  // }

  // if (Ps3.event.button_down.left)
  //   Serial.println("Started pressing the left button");
  // if (Ps3.event.button_up.left)
  //   Serial.println("Released the left button");


  //------------- Digital shoulder button events -------------


  if (Ps3.event.button_down.r1) {
  }
  // Serial.println("Started pressing the right shoulder button");
  if (Ps3.event.button_up.r1)
    // Serial.println("Released the right shoulder button");

    //-------------- Digital trigger button events -------------


    if (Ps3.event.button_down.r2)
      Serial.println("Started pressing the right trigger button");
  if (Ps3.event.button_up.r2)
    Serial.println("Released the right trigger button");





  if (Ps3.event.button_down.ps) {
  } else if (Ps3.event.button_up.ps) {
  }





  //--------------- Analog D-pad button events ----------------
  // if (abs(Ps3.event.analog_changed.button.up)) {
  //   Serial.print("Pressing the up button: ");
  //   Serial.println(Ps3.data.analog.button.up, DEC);
  // }

  // if (abs(Ps3.event.analog_changed.button.right)) {
  //   Serial.print("Pressing the right button: ");
  //   Serial.println(Ps3.data.analog.button.right, DEC);
  // }

  // if (abs(Ps3.event.analog_changed.button.down)) {
  //   Serial.print("Pressing the down button: ");
  //   Serial.println(Ps3.data.analog.button.down, DEC);
  // }
  // if (abs(Ps3.event.analog_changed.button.left)) {
  //   Serial.print("Pressing the left button: ");
  //   Serial.println(Ps3.data.analog.button.left, DEC);
  // }

  // //---------- Analog shoulder/trigger button events ----------


  // if (abs(Ps3.event.analog_changed.button.r1)) {
  //   // Serial.print("Pressing the right shoulder button: ");
  //   // Serial.println(Ps3.data.analog.button.r1, DEC);
  // }



  // if (abs(Ps3.event.analog_changed.button.r2)) {
  //   // Serial.print("Pressing the right trigger button: ");
  //   // Serial.println(Ps3.data.analog.button.r2, DEC);
  // }

  // //---- Analog cross/square/triangle/circle button events ----
  // if (abs(Ps3.event.analog_changed.button.triangle)) {
  //   Serial.print("Pressing the triangle button: ");
  //   Serial.println(Ps3.data.analog.button.triangle, DEC);
  // }

  // if (abs(Ps3.event.analog_changed.button.circle)) {
  //   Serial.print("Pressing the circle button: ");
  //   Serial.println(Ps3.data.analog.button.circle, DEC);
  // }

  // if (abs(Ps3.event.analog_changed.button.cross)) {
  //   Serial.print("Pressing the cross button: ");
  //   Serial.println(Ps3.data.analog.button.cross, DEC);
  // }

  // if (abs(Ps3.event.analog_changed.button.square)) {
  //   Serial.print("Pressing the square button: ");
  //   Serial.println(Ps3.data.analog.button.square, DEC);
  // }

  if ((ULNA + GRIPPER + y) >= 0 && (BASE_HGT + HUMERUS + z) >= 0) {
    if (servo1 < 0) servo1 = 0;
    if (servo2 < 0) servo2 = 0;
    if (servo3 < 0) servo3 = 0;
    if (servo4 < 0) servo4 = 0;
    if (servo1 > 180) servo1 = 180;
    if (servo2 > 180) servo2 = 180;
    if (servo3 > 180) servo3 = 180;
    if (servo4 > 180) servo4 = 180;
    /* 控制每个舵机运行角度*/
    myservo1.write(servo1);  // 设置舵机角度
    myservo2.write(servo2);  // 设置舵机角度
    myservo3.write(servo3);  // 设置舵机角度
    myservo4.write(servo4);  // 设置舵机角度
    myservo5.write(g);       // 设置舵机角度
  } else {
    // (ULNA + GRIPPER + y) = 0;
    // (BASE_HGT + HUMERUS + z) = 0;
    set_arm(x, ULNA + GRIPPER + y, BASE_HGT + HUMERUS + z, w);
    // vibrate = 50;
  }
}

void onConnect() {
  Serial.println("Connected.");
}

void setup() {
  Serial.begin(115200);  // 波特率
  myservo1.attach(BAS);
  myservo2.attach(SHL);
  myservo3.attach(ELB);
  myservo4.attach(WRI);
  myservo5.attach(GRI);

  set_arm(0, ULNA + GRIPPER, BASE_HGT + HUMERUS, 0);  // 逆运动学机械臂位置
  delay(1000);                                        // wait 1s

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("00:00:00:00:00:00");  // MAC地址 必须设置和手柄一致

  delay(1000);  // wait 1s
  Serial.println("Ready.");
  set_arm(0, ULNA + GRIPPER, BASE_HGT + HUMERUS, 0);  // 逆运动学机械臂位置
  delay(1000);                                        // wait 1s
  set_arm(0, ULNA + GRIPPER, BASE_HGT + HUMERUS, 0);  // 逆运动学机械臂位置
  delay(1000);                                        // wait 1s
}

void loop() {
  if (!Ps3.isConnected())
    return;

  //-------------------- Player LEDs -------------------
  //    Serial.print("Setting LEDs to player "); Serial.println(player, DEC);
     Ps3.setPlayer(1);
  //
  //    player += 1;
  //    if(player > 10) player = 0;

  delay(100);
}

/* arm positioning routine utilizing inverse kinematics 利用逆运动学的手臂定位程序*/
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive 
   z是高度，y是从底座中心向外的距离，x是边到边。 y,z只能为正数*/
void set_arm(float x, float y, float z, float grip_angle_d) {
  float grip_angle_r = radians(grip_angle_d);  //grip angle in radians for use in calculations 爪子的弧度，用于计算
  /* Base angle and radial distance from x,y coordinates 距x，y坐标的底角和径向距离*/
  float bas_angle_r = atan2(x, y);
  float rdist = sqrt((x * x) + (y * y));

  /* rdist is y coordinate for the arm -rdist是手臂的y坐标*/
  y = rdist;

  /* Grip offsets calculated based on grip angle 根据握把角度计算的握把偏移量*/
  float grip_off_z = (sin(grip_angle_r)) * GRIPPER;
  float grip_off_y = (cos(grip_angle_r)) * GRIPPER;

  /* Wrist position 手腕位置*/
  // float wrist_z = z - BASE_HGT;
  float wrist_z = (z - grip_off_z) - BASE_HGT;
  float wrist_y = y - grip_off_y;

  /* Shoulder to wrist distance (AKA sw) 肩到腕的距离（又名 sw）*/
  float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
  float s_w_sqrt = sqrt(s_w);

  /* s_w angle to ground s_w与地面的角度*/
  float a1 = atan2(wrist_z, wrist_y);

  /* s_w angle to humerus - s_w与肱骨的角度*/
  float a2 = acos(((hum_sq - uln_sq) + s_w) / (2 * HUMERUS * s_w_sqrt));

  /* shoulder angle 肩角*/
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees(shl_angle_r);

  /* elbow angle 肘角*/
  float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));
  float elb_angle_d = degrees(elb_angle_r);
  float elb_angle_dn = -(180.0 - elb_angle_d);

  /* wrist angle 手腕角度*/
  float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;

  servo1 = BAS_MID - ((degrees(bas_angle_r)));  // bas_servopulse
  servo2 = SHL_MID + (shl_angle_d - 90.0);      // shl_servopulse
  servo3 = ELB_MID - (elb_angle_d - 90.0);      // elb_servopulse
  // servo4 = WRI_MID + wri_angle_d;               // wri_servopulse YF6125双出轴
  servo4 = 180 - (WRI_MID + wri_angle_d);  // wri_servopulse MG996

  /* Servo pulses 伺服脉冲*/
  // float bas_servopulse = 1500.0 - ((degrees(bas_angle_r)) * 11.11);
  // float shl_servopulse = SHL_SERVO_ERROR + ((shl_angle_d - 90.0) * 6.6);
  // float elb_servopulse = ELB_SERVO_ERROR - ((elb_angle_d - 90.0) * 6.6);
  // float wri_servopulse = WRI_SERVO_ERROR + (wri_angle_d * 11.1);
  // float wri_servopulse = WRI_SERVO_ERROR + (elb_servopulse - shl_servopulse);

  /* Set servos */
  // BAS_SERVO.writeMicroseconds(ftl(bas_servopulse));
  // WRI_SERVO.writeMicroseconds(ftl(wri_servopulse));
  // SHL_SERVO.writeMicroseconds(ftl(shl_servopulse));
  // ELB_SERVO.writeMicroseconds(ftl(elb_servopulse));
}
