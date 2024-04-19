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
  @Date: 04/19/2024
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

// 手柄摇杆数值范围设置，保留误差范围，正常手柄不需要修改
// Set the numerical range of the joystick and retain the error range. Normal handles do not need to be modified.
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
#define PWM1A 0  // PWM通道 PWM channel
#define PWM1B 1  // PWM通道 PWM channel
#define PWM2A 2  // PWM通道 PWM channel
#define PWM2B 3  // PWM通道 PWM channel
#define PWM3A 4  // PWM通道 PWM channel
#define PWM3B 5  // PWM通道 PWM channel
#define PWM4A 6  // PWM通道 PWM channel
#define PWM4B 7  // PWM通道 PWM channel

#define Motor1A 6   // 电机调速引脚 Motor speed regulating pin
#define Motor1B 7   // 电机调速引脚 Motor speed regulating pin
#define Motor2A 21  // 电机调速引脚 Motor speed regulating pin
#define Motor2B 18  // 电机调速引脚 Motor speed regulating pin
#define Motor3A 17  // 电机调速引脚 Motor speed regulating pin
#define Motor3B 16  // 电机调速引脚 Motor speed regulating pin
#define Motor4A 9   // 电机调速引脚 Motor speed regulating pin
#define Motor4B 8   // 电机调速引脚 Motor speed regulating pin

// 900hz PWM波频率 根据具体的驱动芯片设置合适的频率值
// PWM frequency, Set the appropriate frequency value according to the specific driver chip
#define freq_PWM 500
#define resolution_PWM 8  // 2的8次方  与arduino保持一致 2 to the 8th power is consistent with arduino

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
  Serial.begin(115200);                        // 波特率 Baud rate
  ledcSetup(PWM1A, freq_PWM, resolution_PWM);  // 通道，频率，分辨率 Channel, Frequency, Resolution
  ledcAttachPin(Motor1A, PWM1A);               // 驱动引脚，通道 Drive Pin, Channel
  ledcSetup(PWM1B, freq_PWM, resolution_PWM);
  ledcAttachPin(Motor1B, PWM1B);

  ledcSetup(PWM2A, freq_PWM, resolution_PWM);  // 通道，频率，分辨率 Channel, Frequency, Resolution
  ledcAttachPin(Motor2A, PWM2A);               // 驱动引脚，通道 Drive Pin, Channel
  ledcSetup(PWM2B, freq_PWM, resolution_PWM);
  ledcAttachPin(Motor2B, PWM2B);

  ledcSetup(PWM3A, freq_PWM, resolution_PWM);  // 通道，频率，分辨率 Channel, Frequency, Resolution
  ledcAttachPin(Motor3A, PWM3A);               // 驱动引脚，通道 Drive Pin, Channel
  ledcSetup(PWM3B, freq_PWM, resolution_PWM);
  ledcAttachPin(Motor3B, PWM3B);

  ledcSetup(PWM4A, freq_PWM, resolution_PWM);  // 通道，频率，分辨率 Channel, Frequency, Resolution
  ledcAttachPin(Motor4A, PWM4A);               // 驱动引脚，通道 Drive Pin, Channel
  ledcSetup(PWM4B, freq_PWM, resolution_PWM);
  ledcAttachPin(Motor4B, PWM4B);

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);  // 手柄配置 PS2 Config
}

void loop() {
  if (error == 1)  // 如果没有找到控制器则跳过循环 skip loop if no controller found
    resetFunc();
  ps2x.read_gamepad(false, vibrate);  // read controller and set large motor to spin at 'vibrate' speed

  // 左侧摇杆Y轴控制小车前进后退，速度值根据摇杆值比例变化
  // The left joystick Y-axis controls the car forward and backward, and the speed value changes in proportion to the joystick value.
  if (PS2MIN <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LF) {  // 前进 Forward
    ledcWrite(PWM1B, LOW);
    ledcWrite(PWM4B, LOW);
    ledcWrite(PWM1A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));
    ledcWrite(PWM4A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));

    ledcWrite(PWM2A, LOW);
    ledcWrite(PWM3A, LOW);
    ledcWrite(PWM2B, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0));
    ledcWrite(PWM3B, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0));
    // Serial.println("UP");
  } else if (PS2MAX >= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) >= PS2MID_LB) {  // 后退 Back
    ledcWrite(PWM1A, LOW);
    ledcWrite(PWM4A, LOW);
    ledcWrite(PWM1B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));
    ledcWrite(PWM4B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));

    ledcWrite(PWM2B, LOW);
    ledcWrite(PWM3B, LOW);
    ledcWrite(PWM2A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
    ledcWrite(PWM3A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
    // Serial.println("DOWN");
  } else if (PS2MIN <= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) <= PS2MID_LL) {  // 左转 Left
    ledcWrite(PWM1B, LOW);
    ledcWrite(PWM4B, LOW);
    ledcWrite(PWM1A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedR, 0));
    ledcWrite(PWM4A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedR, 0));

    ledcWrite(PWM2B, LOW);
    ledcWrite(PWM3B, LOW);
    ledcWrite(PWM2A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedL, 0));
    ledcWrite(PWM3A, map(ps2x.Analog(PSS_LX), PS2MIN, PS2MID_LL, MidSpeedL, 0));
    // Serial.println("LEFT");
  } else if (PS2MAX >= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) >= PS2MID_LR) {  // 右转 Right
    ledcWrite(PWM1A, LOW);
    ledcWrite(PWM4A, LOW);
    ledcWrite(PWM1B, map(ps2x.Analog(PSS_LX), PS2MID_LR, PS2MAX, 0, MidSpeedR));
    ledcWrite(PWM1B, map(ps2x.Analog(PSS_LX), PS2MID_LR, PS2MAX, 0, MidSpeedR));

    ledcWrite(PWM2A, LOW);
    ledcWrite(PWM3A, LOW);
    ledcWrite(PWM2B, map(ps2x.Analog(PSS_LX), PS2MAX, PS2MID_LR, MidSpeedL, 0));
    ledcWrite(PWM3B, map(ps2x.Analog(PSS_LX), PS2MAX, PS2MID_LR, MidSpeedL, 0));
    // Serial.println("RIGHT");
  }

  // 左摇杆前和右摇杆左控制前进状态差速转弯
  // The left joystick forward and the right joystick left control the forward state and differential turning.
  if (PS2MIN <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LF && PS2MIN <= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) <= PS2MID_RL) {
    ledcWrite(PWM1B, LOW);
    ledcWrite(PWM4B, LOW);
    ledcWrite(PWM1A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));
    ledcWrite(PWM4A, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedR, 0));

    ledcWrite(PWM2A, LOW);
    ledcWrite(PWM3A, LOW);
    ledcWrite(PWM2B, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0)));
    ledcWrite(PWM3B, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MIN, PS2MID_LF, MidSpeedL, 0)));
  }

  // 左摇杆前和右摇杆右控制前进状态差速转弯
  // The left joystick forward and the right joystick right control the forward state and differential turning.
  else if (PS2MIN <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LF && PS2MAX >= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) >= PS2MID_RR) {
    ledcWrite(PWM1B, LOW);
    ledcWrite(PWM4B, LOW);
    ledcWrite(PWM1A, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedR, 0)));
    ledcWrite(PWM4A, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedR, 0)));

    ledcWrite(PWM2A, LOW);
    ledcWrite(PWM3A, LOW);
    ledcWrite(PWM2B, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedL, 0));
    ledcWrite(PWM3B, map(ps2x.Analog(PSS_LY), PS2MAX, PS2MID_LF, MidSpeedL, 0));
  }

  // 左摇杆后和右摇杆左控制前进状态差速转弯
  // Left stick rear and right stick left control forward status differential turning
  else if (PS2MAX >= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) >= PS2MID_LB && PS2MIN <= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) <= PS2MID_RL) {
    ledcWrite(PWM1A, LOW);
    ledcWrite(PWM4A, LOW);
    ledcWrite(PWM1B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));
    ledcWrite(PWM4B, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR));

    ledcWrite(PWM2B, LOW);
    ledcWrite(PWM3B, LOW);
    ledcWrite(PWM2A, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL)));
    ledcWrite(PWM3A, map(ps2x.Analog(PSS_RX), PS2MIN, PS2MID_RL, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL)));
  }

  // 左摇杆后和右摇杆右控制前进状态差速转弯
  // Left stick rear and right stick right control forward status differential turning
  else if (PS2MAX >= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) >= PS2MID_LB && PS2MAX >= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) >= PS2MID_RR) {
    ledcWrite(PWM1A, LOW);
    ledcWrite(PWM4A, LOW);
    ledcWrite(PWM1B, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR)));
    ledcWrite(PWM4B, map(ps2x.Analog(PSS_RX), PS2MAX, PS2MID_RR, 0, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedR)));

    ledcWrite(PWM2B, LOW);
    ledcWrite(PWM3B, LOW);
    ledcWrite(PWM2A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
    ledcWrite(PWM3A, map(ps2x.Analog(PSS_LY), PS2MID_LB, PS2MAX, 0, MidSpeedL));
  }

  // 左摇杆居中停止 Left joystick stops in the center
  if (PS2MID_LF <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LB
      && PS2MID_LL <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= PS2MID_LR) {
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
  ledcWrite(PWM1A, HIGH);
  ledcWrite(PWM1B, HIGH);
  ledcWrite(PWM2A, HIGH);
  ledcWrite(PWM2B, HIGH);
  ledcWrite(PWM3A, HIGH);
  ledcWrite(PWM3B, HIGH);
  ledcWrite(PWM4A, HIGH);
  ledcWrite(PWM4B, HIGH);
}
