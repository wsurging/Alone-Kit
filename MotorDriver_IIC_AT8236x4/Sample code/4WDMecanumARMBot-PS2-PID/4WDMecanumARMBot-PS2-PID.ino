/*********************************************************************
  麦克纳姆轮减震小车PS2控制程序，含PID
  Mecanum wheel shock-absorbing trolley PS2 control program, including PID

  摇杆和按键可以控制小车的前进后退，旋转；摇杆可以通过组合模式实现全方位运行。
  SELECT按键恢复初始速度值。按键部分控制机械臂，使用逆运动学控制方式。
  The joystick and buttons can control the forward, backward, and rotation of the car; 
  the joystick can achieve all-round operation through combination mode.
  SELECT button restores the initial speed value. 
  The button part controls the robotic arm and uses inverse kinematics control.

   电机分布:  M2     M1   Motor distribution
              \     /

              /     \
             M3     M4

  注意事项：
  1.舵机驱动时需将频率设置为50Hz，语句pwm.setPWMFreq(50);
  2.当供电电压不是3S（11.1v）电池的时候需要适当的调节PID参数，做速度值的适配。
  3.当需要驱动舵机时需要给接线端子连接电源，电源参考舵机电压范围，一般支持4.8-7.2v。
  Precautions:
   1. When driving the servo, the frequency needs to be set to 50Hz, using the statement pwm.setPWMFreq(50);
   2. When the power supply voltage is not a 3S (11.1v) battery, the PID parameters need to be adjusted appropriately to adapt the speed value.
   3. When you need to drive the servo, you need to connect the power supply to the terminal block. 
   The power supply refers to the servo voltage range, which generally supports 4.8-7.2v.

  供电方式：
  1.电机和主板电压最大支持12v,不建议超过此电压，会有损坏电机和主板的风险。端子VIN，GND。
  2.舵机使用5v降压供电，端口VS，GND，电流建议5A以上。
  Power supply:
   1. The maximum voltage of the motor and motherboard supports 12v. It is not recommended to exceed this voltage because there is a risk of damaging the motor and motherboard. Terminal VIN, GND.
   2. The servo uses 5v step-down power supply, ports VS, GND, and the current is recommended to be above 5A.
 
  @Author: Wsurging
  @Version: V1.0
  @Date: 04/18/2024
**********************************************************************/
#include <PID_v1.h>              // libraries
#include <PS2X_lib.h>            // libraries
#include <PinChangeInterrupt.h>  // libraries v1.2.9
#include <MotorDriver.h>         // v0.08

#define UARTDEBUG  // 注释掉串口不打印数据 Comment out the serial port and do not print data

/**********************************************************************
              IIC电机驱动配置 IIC motor drive configuration
**********************************************************************/
#define MOTORTYPE YF_IIC_RZ  // rz7889 驱动型号选择  Driver model selection
uint8_t SerialDebug = 1;

// 设置电机旋转方向1为正传，-1为反转
// Set the motor rotation direction to 1 for forward rotation and -1 for reverse rotation.
const int offsetm1 = 1;
const int offsetm2 = -1;
const int offsetm3 = -1;
const int offsetm4 = 1;

MotorDriver motorDriver = MotorDriver(MOTORTYPE);

/**********************************************************************
                 PS2无线手柄 PS2 wireless controller
**********************************************************************/
PS2X ps2x;

#define PS2_DAT 10  // DAT端口接线 DAT port wiring
#define PS2_CMD 11  // CMD端口接线 CMD port wiring
#define PS2_CS 12   // CS端口接线 CS port wiring
#define PS2_CLK 14  // CLK端口接线 CLK port wiring

#define pressures false  // 按键模拟值关闭 Key simulation value off
#define rumble true      // 手柄震动打开 The handle vibrates to open

int speedm_x = 0;  // 左摇杆X轴 Left joystick X axis
int speedm_y = 0;  // 左摇杆Y轴 Left joystick Y axis
int speedm_w = 0;  // 右摇杆X轴 Right joystick X axis

int error = 0;
byte type = 0;
byte vibrate = 0;

/**********************************************************************
                       编码器电机配置与PID函数
**********************************************************************/
int MAXSPEED = 4000;    // 小车电机速度最大值--最大4096 Maximum speed--maximum 4096
int MINSPEED = 300;     // 小车电机速度最小值 Minimum speed of car motor
int MOTORSPEED = 2000;  // 小车速度默认初始值 The default initial value of the car speed
int stime = 50;         // 采样时间 sampling time

float val = 23;  // 编码器检测调节参数 Encoder detection adjustment parameters
float rpm = 0;

int MIDSPEED = MOTORSPEED;
int M1Speed = MOTORSPEED;
int M2Speed = MOTORSPEED;
int M3Speed = MOTORSPEED;
int M4Speed = MOTORSPEED;

// 编码器检测引脚，A对应电机黄线，B对应电机白线 arduino UNO/NANO数字接口
// Encoder detection pin, A corresponds to the motor yellow wire, B corresponds to the motor white wire
#define M1EncoderA 3  // 电机1A相
#define M1EncoderB 5  // 电机1B相
#define M2EncoderA 2  // 电机2A相
#define M2EncoderB 4  // 电机2B相
#define M3EncoderA 7  // 电机3A相
#define M3EncoderB 6  // 电机3B相
#define M4EncoderA 9  // 电机4A相
#define M4EncoderB 8  // 电机4B相

//PID 参数，调整 PID 以适应电机 PID parameters, adjust PID to suit the motor
double Kp1 = 0.8, Ki1 = 3, Kd1 = 0.002;
double Kp2 = 0.8, Ki2 = 3, Kd2 = 0.002;
double Kp3 = 0.8, Ki3 = 3, Kd3 = 0.002;
double Kp4 = 0.8, Ki4 = 3, Kd4 = 0.002;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;

PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);
PID myPID3(&Input3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);
PID myPID4(&Input4, &Output4, &Setpoint4, Kp4, Ki4, Kd4, DIRECT);

// 编码器检测值 Encoder detection value
volatile double encoderPos1 = 0;
volatile double encoderPos2 = 0;
volatile double encoderPos3 = 0;
volatile double encoderPos4 = 0;

/**********************************************************************
                    机械臂配置 Robot arm configuration
**********************************************************************/
int servoSpeed = 4;  // 舵机运行速度 seervo speed

// Arm dimensions(mm) 关节长度（毫米）
#define BASE_HGT 150  // 底座高度 Base height
#define HUMERUS 105   // 肩部到肘部 shoulder to elbow
#define ULNA 128      // 肘部到手腕 elbow to wrist
#define GRIPPER 125   // 夹具（手腕旋转机构）Clamp (wrist rotation mechanism)

// Define right angle errors (90 degree value (us))定义直角误差
#define BAS_MID 90
#define SHL_MID 92
#define ELB_MID 95
#define WRI_MID 95
#define GRI_MID 90
#define GRI_MIN 30
int g = 90;

float SERVO1, SERVO2, SERVO3, SERVO4;
float x, y, z, w;

#define ftl(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x)-0.5))  //float to long conversion 浮点数到长整型的转换

/* pre-calculations 预先计算*/
float hum_sq = HUMERUS * HUMERUS;
float uln_sq = ULNA * ULNA;

void (*resetFunc)(void) = 0;  // PS2 Restart

void setup() {
  Serial.begin(115200);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_CS, PS2_DAT, pressures, rumble);

  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  // When controlling the servo, you need to set the PWM frequency ~50
  // 控制舵机时，需要设置PWM频率 ~50
  motorDriver.setPWMFreq(50);

  for (int i = 2; i <= 9; i++) pinMode(i, INPUT_PULLUP);
  // 引脚变化中断 pin change interrupt
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M1EncoderA), M1Encoder, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M2EncoderA), M2Encoder, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M3EncoderA), M3Encoder, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M4EncoderA), M4Encoder, RISING);
  myPID1.SetMode(AUTOMATIC);  // 自动开启 Automatically turn on
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
  myPID4.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-4096, 4096);  // 限制速度范围 Limit speed range
  myPID2.SetOutputLimits(-4096, 4096);
  myPID3.SetOutputLimits(-4096, 4096);
  myPID4.SetOutputLimits(-4096, 4096);
  // 给定手爪坐标set_arm(左右(±), 前后(+), 高度(+), 手爪俯仰角度(±))
  // Given the hand paw coordinates set_arm (left and right (±), front and back (+), height (+), hand paw pitch angle (±))
  set_arm(0, ULNA, BASE_HGT + HUMERUS, 0);
  motorDriver.servoWrite(S5, GRI_MID);
  delay(1000);
}

void loop() {
  if (error == 1) {  //skip loop if no controller found
    Serial.println("ERROR!!");
    delay(10);
    resetFunc();
  }
  ps2x.read_gamepad(false, vibrate);  // 不可删除

  // 按键控制机械臂运行 Buttons control the operation of the robotic arm
  if (ps2x.Button(PSB_PAD_UP)) {  // 机械臂向前伸展 Robotic arm extends forward
    y += servoSpeed;

    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_PAD_RIGHT)) {  // 机械臂向右旋转 The robotic arm rotates to the right
    x -= servoSpeed;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_PAD_LEFT)) {  // 机械臂向左旋转 The robotic arm rotates to the left
    x += servoSpeed;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_PAD_DOWN)) {  // 机械臂向后运行 Robotic arm runs backwards
    y -= servoSpeed;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_TRIANGLE)) {  // 三角形 手腕上升 triangle wrist rising
    w += servoSpeed;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_CROSS)) {  // 叉  手腕下降 cross wrist down
    w -= servoSpeed;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_R1)) {  // R1 机械臂抬起 R1 robotic arm raised
    z += servoSpeed;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_R2)) {  // R2 机械臂下降 R2 robotic arm descends
    z -= servoSpeed;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
  }

  if (ps2x.Button(PSB_L1)) {  // 手爪夹紧 Claw clamping
    g -= 5;
    if (g < GRI_MIN) {
      g = GRI_MIN;
      vibrate = 100;
    }
  }

  if (ps2x.Button(PSB_L2)) {  // 手爪松开 Release the claws
    g += 5;
    if (g > GRI_MID) {
      g = GRI_MID;
      vibrate = 100;
    }
  }

  if (ps2x.Button(PSB_CIRCLE)) {  // 圆圈 无
  }

  if (ps2x.Button(PSB_SQUARE)) {  // 正方形 无
  }

  // 按下L3电机速度增加300，最大到MAXSPEED值 Press L3 to increase the motor speed by 300, up to the MAXSPEED value.
  if (ps2x.ButtonPressed(PSB_L3)) {
    MOTORSPEED += 300;
    vibrate = 200;
    if (MOTORSPEED > MAXSPEED) {
      MOTORSPEED = MAXSPEED;
      vibrate = 200;
    }
  }

  // 按下R3电机速度减少300，最低到MINSPEED值 Press R3 to reduce the motor speed by 300, to the lowest value of MINSPEED
  if (ps2x.ButtonPressed(PSB_R3)) {
    MOTORSPEED -= 300;
    vibrate = 200;
    if (MOTORSPEED < MINSPEED) {
      MOTORSPEED = MINSPEED;
      vibrate = 200;
    }
  }

  // 按下选择键-复位初始速度值 Press the select key - reset initial speed value
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    MOTORSPEED = MIDSPEED;
    x = 0;
    y = 0;
    z = 0;
    w = 0;
    g = GRI_MID;
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
    motorDriver.servoWrite(S5, g);
    vibrate = 100;
#ifdef UARTDEBUG
    Serial.print("reset");
#endif
  }
  if (ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonReleased(PSB_L2) || ps2x.ButtonReleased(PSB_R2) || ps2x.ButtonReleased(PSB_L3) || ps2x.ButtonReleased(PSB_R3)
      || ps2x.ButtonReleased(PSB_PAD_UP) || ps2x.ButtonReleased(PSB_PAD_DOWN) || ps2x.ButtonReleased(PSB_PAD_RIGHT) || ps2x.ButtonReleased(PSB_PAD_LEFT)
      || ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CIRCLE) || ps2x.ButtonReleased(PSB_CROSS) || ps2x.ButtonReleased(PSB_SELECT)) {
    vibrate = 0;
  }

  if (SERVO1 >= 0 && SERVO1 <= 180 && SERVO2 >= 0 && SERVO2 <= 180 && SERVO3 >= 0 && SERVO3 <= 180 && SERVO4 >= 0 && SERVO4 <= 180) {
    motorDriver.servoWrite(S1, SERVO1);
    motorDriver.servoWrite(S2, SERVO2);
    motorDriver.servoWrite(S3, SERVO3);
    motorDriver.servoWrite(S4, SERVO4);
    motorDriver.servoWrite(S5, g);

#ifdef UARTDEBUG
    set_arm(x, ULNA + y, BASE_HGT + HUMERUS + z, w);
    Serial.print("X.Y.Z.W: ");
    Serial.print(x);
    Serial.print(" : ");
    Serial.print(ULNA + y);
    Serial.print(" : ");
    Serial.print(BASE_HGT + HUMERUS + z);
    Serial.print(" : ");
    Serial.print(w);
    Serial.print("  -BAS:");
    Serial.print(SERVO1);
    Serial.print("   WRI:");
    Serial.print(SERVO2);
    Serial.print("   SHL:");
    Serial.print(SERVO3);
    Serial.print("   ELB:");
    Serial.print(SERVO4);
    Serial.print("   GRI:");
    Serial.print(g);
#endif
  } else {
    vibrate = 100;
  }

  /********************************************************************************************************
                                      摇杆控制部分 Joystick control part
    左侧摇杆和右侧X轴（横向）摇杆联动控制电机运行，可控制电机全方向运动，每个摇杆也可以单独控制小车运行，速度比例控制。
    The left rocker and the right X-axis (lateral) rocker control the operation of the motor in conjunction,
    and can control the movement of the motor in all directions. Each rocker can also independently control the operation of the car, 
    and the speed is proportional control.
  *********************************************************************************************************/
  if (ps2x.Analog(PSS_LY) < 120 || ps2x.Analog(PSS_LY) > 135 || ps2x.Analog(PSS_LX) < 120 || ps2x.Analog(PSS_LX) > 135 || ps2x.Analog(PSS_RX) < 120 || ps2x.Analog(PSS_RX) > 135) {
    if (ps2x.Analog(PSS_LY) < 120 && ps2x.Analog(PSS_LY) >= 0) {
      speedm_y = map(ps2x.Analog(PSS_LY), 0, 120, MOTORSPEED, 0);
    } else if (ps2x.Analog(PSS_LY) > 135 && ps2x.Analog(PSS_LY) <= 255) {
      speedm_y = map(ps2x.Analog(PSS_LY), 135, 255, 0, -MOTORSPEED);
    } else
      speedm_y = 0;

    if (ps2x.Analog(PSS_LX) < 120 && ps2x.Analog(PSS_LX) >= 0) {
      speedm_x = map(ps2x.Analog(PSS_LX), 0, 120, MOTORSPEED, 0);
    } else if (ps2x.Analog(PSS_LX) > 135 && ps2x.Analog(PSS_LX) <= 255) {
      speedm_x = map(ps2x.Analog(PSS_LX), 135, 255, 0, -MOTORSPEED);
    } else
      speedm_x = 0;

    if (ps2x.Analog(PSS_RX) < 120 && ps2x.Analog(PSS_RX) >= 0) {
      speedm_w = map(ps2x.Analog(PSS_RX), 0, 120, MOTORSPEED, 0);
    } else if (ps2x.Analog(PSS_RX) > 135 && ps2x.Analog(PSS_RX) <= 255) {
      speedm_w = map(ps2x.Analog(PSS_RX), 135, 255, 0, -MOTORSPEED);
    } else
      speedm_w = 0;

    // 设定值，根据摇杆范围设定 Setting value, set according to the joystick range
    Setpoint1 = speedm_y + speedm_x + speedm_w;
    Setpoint2 = speedm_y - speedm_x - speedm_w;
    Setpoint3 = speedm_y + speedm_x - speedm_w;
    Setpoint4 = speedm_y - speedm_x + speedm_w;

    // 编码器检测做输入值,编码器采样和采样时间，脉冲数量，转速有关系；乘以val以映射到速度值范围
    // Encoder detection is used as input value. Encoder sampling is related to sampling time, number of pulses, and speed; multiply by val to map to the speed value range.
    Input1 = encoderPos1 * val;
    Input2 = encoderPos2 * val;
    Input3 = encoderPos3 * val;
    Input4 = encoderPos4 * val;

    //转速=单位时间内获取的脉冲*单位时间（换算成秒）/磁环精度cpr/减速比/车轮一圈距离(mm)
    rpm = abs(encoderPos2) * (1000 / stime) / 12 / 74.8 * (3.14 * 76 / 1000);

    myPID1.Compute();  // PID计算
    myPID2.Compute();
    myPID3.Compute();
    myPID4.Compute();

    motorDriver.setMotor(Output1, Output2, Output3, Output4);  // 电机经过PID计算后运行 The motor runs after PID calculation

    encoderPos1 = 0;  // 编码器值清零 Encoder value cleared
    encoderPos2 = 0;
    encoderPos3 = 0;
    encoderPos4 = 0;

  } else {
    motorDriver.stopMotor(MAll);  // 释放摇杆 刹车  Stop
  }

#ifdef UARTDEBUG
  Serial.print("   -Set1:");
  Serial.print(Setpoint1);
  Serial.print("  In1:");
  Serial.print(Input1);
  Serial.print("  Out1:");
  Serial.print(Output1);
  Serial.print("  Speed:");
  Serial.print(MOTORSPEED);
  Serial.print("  rpm:");
  Serial.print(rpm);
  Serial.println();
#endif
  delay(stime);  // 采样时间，不可随意修改 Sampling time cannot be modified at will
}

/* arm positioning routine utilizing inverse kinematics 利用逆运动学的手臂定位程序*/
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive 
z 是高度，y 是从底座中心向外的距离，x 是边到边。 y,z 只能为正数*/
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

  SERVO1 = BAS_MID - ((degrees(bas_angle_r)));  // bas_servopulse
  SERVO2 = SHL_MID + (shl_angle_d - 90.0);      // shl_servopulse
  SERVO3 = ELB_MID - (elb_angle_d - 90.0);      // elb_servopulse
  SERVO4 = WRI_MID + wri_angle_d;               // wri_servopulse

  /* Servo pulses 伺服脉冲*/
  // float bas_servopulse = 1500.0 - ((degrees(bas_angle_r)) * 11.11);
  // float shl_servopulse = SHL_SERVO_ERROR + ((shl_angle_d - 90.0) * 6.6);
  // float elb_servopulse = ELB_SERVO_ERROR - ((elb_angle_d - 90.0) * 6.6);
  // // float wri_servopulse = WRI_SERVO_ERROR + (wri_angle_d * 11.1);
  // float wri_servopulse = WRI_SERVO_ERROR + (elb_servopulse - shl_servopulse);

  /* Set servos */
  // BAS_SERVO.writeMicroseconds(ftl(bas_servopulse));
  // WRI_SERVO.writeMicroseconds(ftl(wri_servopulse));
  // SHL_SERVO.writeMicroseconds(ftl(shl_servopulse));
  // ELB_SERVO.writeMicroseconds(ftl(elb_servopulse));
}

// 编码器计数，并通过判断AB位置的先后顺序来检测方向。
// The encoder counts and detects the direction by judging the sequence of AB positions.
void M1Encoder() {
  if (digitalRead(M1EncoderA) == HIGH) {
    if (digitalRead(M1EncoderB) == LOW) {
      encoderPos1--;
    } else {
      encoderPos1++;
    }
  } else {
    if (digitalRead(M1EncoderB) == LOW) {
      encoderPos1++;
    } else {
      encoderPos1--;
    }
  }
}

void M2Encoder() {
  if (digitalRead(M2EncoderA) == HIGH) {
    if (digitalRead(M2EncoderB) == LOW) {
      encoderPos2++;
    } else {
      encoderPos2--;
    }
  } else {
    if (digitalRead(M2EncoderB) == LOW) {
      encoderPos2--;
    } else {
      encoderPos2++;
    }
  }
}

void M3Encoder() {
  if (digitalRead(M3EncoderA) == HIGH) {
    if (digitalRead(M3EncoderB) == LOW) {
      encoderPos3++;
    } else {
      encoderPos3--;
    }
  } else {
    if (digitalRead(M3EncoderB) == LOW) {
      encoderPos3--;
    } else {
      encoderPos3++;
    }
  }
}

void M4Encoder() {
  if (digitalRead(M4EncoderA) == HIGH) {
    if (digitalRead(M4EncoderB) == LOW) {
      encoderPos4--;
    } else {
      encoderPos4++;
    }
  } else {
    if (digitalRead(M4EncoderB) == LOW) {
      encoderPos4++;
    } else {
      encoderPos4--;
    }
  }
}