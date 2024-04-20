/**********************************************************************
  4WD_OffRoadRobot_ESP32_S2_PS2

  硬件型号：主控板：ESP32 S2 MINI
           电机驱动芯片：RZ7886
           电源：聚合物锂电池2s
           电机：GA25-370  21：1减速比
  Hardware model: Main control board: ESP32 S2 MINI
                  Motor driver chip: RZ7886
                  Power supply: polymer lithium battery 2s
                  Motor: GA25-370 21:1 reduction ratio

  控制说明：
  1.左摇杆前后控制小车前进后退，左右控制原地转向。
  2.右摇杆左右和左摇杆前后一起控制实现差速转向。
  3.摇杆左右按键控制左右侧轮子速度值减小。
  4.按键L1,L2设置小车速度值增大减小，SELECT恢复速度值设置。

  Control instructions:
   1. The left joystick controls the car forward and backward, and the left and right joystick controls the steering in place.
   2. The right joystick left and right and the left joystick front and back are controlled together to achieve differential steering.
   3. Use the left and right buttons of the joystick to control the speed value of the left and right wheels to decrease.
   4. Press keys L1 and L2 to set the speed value of the car to increase or decrease, and SELECT to restore the speed value setting.

  @Author: Wsurging
  @Version: V1.0
  @Date: 04/16/2024
**********************************************************************/
#include <PS2X_lib.h>

PS2X ps2x;

/**********************************************************************
   Set PS2 Pin
   设置PS2手柄接线端口，默认设置，不能更改
 *********************************************************************/
#define PS2_DAT 5
#define PS2_CMD 4  // PS2 pin
#define PS2_SEL 3
#define PS2_CLK 2

#define PS2MIN 0       // 摇杆最小值 Rocker minimum value
#define PS2MAX 255     // 摇杆最大值 Rocker maximum value
#define PS2MID_LF 120  // 左摇杆Y轴中间值 (0-127)   Middle value of left joystick Y axis
#define PS2MID_LB 135  // 左摇杆Y轴中间值 (128-255) Middle value of left joystick Y axis
#define PS2MID_LL 120  // 左摇杆X轴中间值 (0-128)   The middle value of the X-axis of the right joystick
#define PS2MID_LR 135  // 左摇杆X轴中间值 (129-255) The middle value of the X-axis of the right joystick
#define PS2MID_RL 120  // 右摇杆X轴中间值 (0-128)   The middle value of the X-axis of the right joystick
#define PS2MID_RR 135  // 右摇杆X轴中间值 (129-255) The middle value of the X-axis of the right joystick

/**********************************************************************
   Set Motor Pin
   设置电机驱动参数，默认设置，不能更改
 *********************************************************************/
#define PWM1A 6   // 电机调速引脚 Motor speed regulating pin
#define PWM1B 7   // 电机调速引脚 Motor speed regulating pin
#define PWM2A 21  // 电机调速引脚 Motor speed regulating pin
#define PWM2B 18  // 电机调速引脚 Motor speed regulating pin
#define PWM3A 17  // 电机调速引脚 Motor speed regulating pin
#define PWM3B 16  // 电机调速引脚 Motor speed regulating pin
#define PWM4A 9   // 电机调速引脚 Motor speed regulating pin
#define PWM4B 8   // 电机调速引脚 Motor speed regulating pin

int MAXSPEEDL = 255;  //摇杆控制左边电机速度，最大255
int MAXSPEEDR = 255;  //摇杆控制右边电机速度，最大255

int MidSpeedL = MAXSPEEDL;
int MidSpeedR = MAXSPEEDR;

#define pressures false  // 按键模拟值 Key analog value
#define rumble true      // 手柄震动 Vibration motor
int error = 0;
byte type = 0;
byte vibrate = 0;
void (*resetFunc)(void) = 0;  // 重启 reset

void setup() {
  Serial.begin(115200);  // 波特率 Baud rate

  pinMode(PWM1A, OUTPUT);  // 设置端口为输出模式 Set the port to output mode
  pinMode(PWM1B, OUTPUT);  // 设置端口为输出模式 Set the port to output mode
  pinMode(PWM2A, OUTPUT);  // 设置端口为输出模式 Set the port to output mode
  pinMode(PWM2B, OUTPUT);  // 设置端口为输出模式 Set the port to output mode
  pinMode(PWM3A, OUTPUT);  // 设置端口为输出模式 Set the port to output mode
  pinMode(PWM3B, OUTPUT);  // 设置端口为输出模式 Set the port to output mode
  pinMode(PWM4A, OUTPUT);  // 设置端口为输出模式 Set the port to output mode
  pinMode(PWM4B, OUTPUT);  // 设置端口为输出模式 Set the port to output mode

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);  // 手柄配置 PS2 Config
}

void loop() {
  if (error == 1)  // 如果没有找到控制器则跳过循环 skip loop if no controller found
    resetFunc();
  ps2x.read_gamepad(false, vibrate);  // read controller and set large motor to spin at 'vibrate' speed

  // 左侧摇杆Y轴控制小车前进后退，速度值根据摇杆值比例变化
  // The left joystick Y-axis controls the car forward and backward, and the speed value changes in proportion to the joystick value.
  if (PS2MIN <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LF) {  // 前进 FORWARD
    digitalWrite(PWM1B, LOW);
    digitalWrite(PWM4B, LOW);
    analogWrite(PWM1A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));
    analogWrite(PWM4A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));

    digitalWrite(PWM2A, LOW);
    digitalWrite(PWM3A, LOW);
    analogWrite(PWM2B, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0));
    analogWrite(PWM3B, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0));
    // Serial.println("UP");
  } else if (PS2MAX >= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) >= PS2MID_LB) {  // 后退 Back
    digitalWrite(PWM1A, LOW);
    digitalWrite(PWM4A, LOW);
    analogWrite(PWM1B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));
    analogWrite(PWM4B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));

    digitalWrite(PWM2B, LOW);
    digitalWrite(PWM3B, LOW);
    analogWrite(PWM2A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
    analogWrite(PWM3A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
    // Serial.println("DOWN");
  } else if (PS2MIN <= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) <= PS2MID_LL) {  // 左转 LEFT
    digitalWrite(PWM1B, LOW);
    digitalWrite(PWM4B, LOW);
    analogWrite(PWM1A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedR, 0));
    analogWrite(PWM4A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedR, 0));

    digitalWrite(PWM2B, LOW);
    digitalWrite(PWM3B, LOW);
    analogWrite(PWM2A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedL, 0));
    analogWrite(PWM3A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedL, 0));
    // Serial.println("LEFT");
  } else if (PS2MAX >= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) >= PS2MID_LR) {  // 右转 RIGHT
    digitalWrite(PWM1A, LOW);
    digitalWrite(PWM4A, LOW);
    analogWrite(PWM1B, map(ps2x.Analog(PSS_LX), PS2MID_LR, PS2MAX, 0, MidSpeedR));
    analogWrite(PWM1B, map(ps2x.Analog(PSS_LX), PS2MID_LR, PS2MAX, 0, MidSpeedR));

    digitalWrite(PWM2A, LOW);
    digitalWrite(PWM3A, LOW);
    analogWrite(PWM2B, map(ps2x.Analog(PSS_LX), PS2MAX, PS2MID_LR, MidSpeedL, 0));
    analogWrite(PWM3B, map(ps2x.Analog(PSS_LX), PS2MAX, PS2MID_LR, MidSpeedL, 0));
    // Serial.println("RIGHT");
  }

  // 左摇杆前和右摇杆左控制前进状态差速转弯
  // The left joystick forward and the right joystick left control the forward state and differential turning.
  if (PS2MIN <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LF && PS2MIN <= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) <= PS2MID_RL) {
    digitalWrite(PWM1B, LOW);
    digitalWrite(PWM4B, LOW);
    analogWrite(PWM1A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));
    analogWrite(PWM4A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));

    digitalWrite(PWM2A, LOW);
    digitalWrite(PWM3A, LOW);
    analogWrite(PWM2B, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0)));
    analogWrite(PWM3B, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0)));
  }

  // 左摇杆前和右摇杆右控制前进状态差速转弯
  // The left joystick forward and the right joystick right control the forward state and differential turning.
  else if (PS2MIN <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LF && PS2MAX >= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) >= PS2MID_RR) {
    digitalWrite(PWM1B, LOW);
    digitalWrite(PWM4B, LOW);
    analogWrite(PWM1A, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedR, 0)));
    analogWrite(PWM4A, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedR, 0)));

    digitalWrite(PWM2A, LOW);
    digitalWrite(PWM3A, LOW);
    analogWrite(PWM2B, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedL, 0));
    analogWrite(PWM3B, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedL, 0));
  }

  // 左摇杆后和右摇杆左控制前进状态差速转弯
  // Left stick rear and right stick left control forward status differential turning
  else if (PS2MAX >= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) >= PS2MID_LB && PS2MIN <= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) <= PS2MID_RL) {
    digitalWrite(PWM1A, LOW);
    digitalWrite(PWM4A, LOW);
    analogWrite(PWM1B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));
    analogWrite(PWM4B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));

    digitalWrite(PWM2B, LOW);
    digitalWrite(PWM3B, LOW);
    analogWrite(PWM2A, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL)));
    analogWrite(PWM3A, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL)));
  }

  // 左摇杆后和右摇杆右控制前进状态差速转弯
  // Left stick rear and right stick right control forward status differential turning
  else if (PS2MAX >= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) >= PS2MID_LB && PS2MAX >= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) >= PS2MID_RR) {
    digitalWrite(PWM1A, LOW);
    digitalWrite(PWM4A, LOW);
    analogWrite(PWM1B, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR)));
    analogWrite(PWM4B, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR)));

    digitalWrite(PWM2B, LOW);
    digitalWrite(PWM3B, LOW);
    analogWrite(PWM2A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
    analogWrite(PWM3A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
  }

  // 左摇杆居中停止 Left joystick stops in the center
  if (PS2MID_LF <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LB
      && PS2MID_LL <= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) <= PS2MID_LR) {
    Stop();
  }

  // 左摇杆按键按下一次减小左侧速度值5
  // Press the left joystick button once to decrease the left speed value by 5
  if (ps2x.ButtonPressed(PSB_L3)) {
    MidSpeedL -= 5;
    if (MidSpeedL < 0) {
      MidSpeedL = 0;
    }
    vibrate = 100;
  }

  // 右摇杆按键按下一次减小右侧速度值5
  // Press the right joystick button once to decrease the right speed value by 5
  if (ps2x.ButtonPressed(PSB_R3)) {
    MidSpeedR -= 5;
    if (MidSpeedR <= 0) {
      MidSpeedR = 0;
    }
    vibrate = 100;
  }

  // 恢复电机初始速度 Restore motor initial speed
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    MidSpeedL = MAXSPEEDL;
    MidSpeedR = MAXSPEEDR;
    vibrate = 100;
  }

  // 按键L1按下一次增加速度20 Pressing the button once increases the speed by 20
  if (ps2x.ButtonPressed(PSB_L1)) {
    MidSpeedL += 20;
    MidSpeedR += 20;
    if (MidSpeedL >= 255) {
      MidSpeedL = 255;
    }
    if (MidSpeedR >= 255) {
      MidSpeedR = 255;
    }
    vibrate = 100;
  }

  // 按键L2按下一次减少速度20 Pressing the button once reduces the speed by 20
  if (ps2x.ButtonPressed(PSB_L2)) {
    MidSpeedL -= 20;
    MidSpeedR -= 20;
    if (MidSpeedL <= 20) {
      MidSpeedL = 20;
    }
    if (MidSpeedR <= 20) {
      MidSpeedR = 20;
    }
    vibrate = 100;
  }

  // 按键松开停止震动 Release the button to stop vibration
  if (ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonReleased(PSB_L2) || ps2x.ButtonReleased(PSB_R2) || ps2x.ButtonReleased(PSB_L3) || ps2x.ButtonReleased(PSB_R3)
      || ps2x.ButtonReleased(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN) || ps2x.ButtonReleased(PSB_PAD_LEFT) || ps2x.ButtonReleased(PSB_PAD_RIGHT)
      || ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CIRCLE) || ps2x.ButtonReleased(PSB_CROSS) || ps2x.ButtonReleased(PSB_SELECT)) {
    vibrate = 0;
  }
  delay(50);
}

void Stop() {  // 停止
  analogWrite(PWM1A, LOW);
  analogWrite(PWM1B, LOW);
  analogWrite(PWM2A, LOW);
  analogWrite(PWM2B, LOW);
  analogWrite(PWM3A, LOW);
  analogWrite(PWM3B, LOW);
  analogWrite(PWM4A, LOW);
  analogWrite(PWM4B, LOW);
}
