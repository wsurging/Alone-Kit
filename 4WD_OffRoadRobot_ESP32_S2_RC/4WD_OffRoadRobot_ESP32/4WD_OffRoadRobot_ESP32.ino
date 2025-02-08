/**********************************************************************
  4WD_OffRoadRobot_ESP32_S2_RC

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

/**********************************************************************
  Set RC Pin
  设置RC接线端口，默认设置，不能更改
 *********************************************************************/
int ch1, ch2, ch3, ch4, ch5, ch6, ch7;

#define CH1_PIN 5
#define CH2_PIN 4
#define CH3_PIN 3
#define CH4_PIN 2
#define CH5_PIN 12
#define CH6_PIN 13

// 手柄摇杆数值范围设置，保留误差范围，正常手柄不需要修改
// Set the numerical range of the joystick and retain the error range. Normal handles do not need to be modified.
#define RCMIN 1000     // 摇杆最小值 Rocker minimum value
#define RCMAX 2000     // 摇杆最大值 Rocker maximum value
#define RCMID_LF 1450  // 左摇杆Y轴中间值 (200-1000)   Middle value of left joystick Y axis
#define RCMID_LB 1550  // 左摇杆Y轴中间值 (1000-1800) Middle value of left joystick Y axis
#define RCMID_LL 1450  // 左摇杆X轴中间值 (200-1000)   The middle value of the X-axis of the right joystick
#define RCMID_LR 1550  // 左摇杆X轴中间值 (1000-1800) The middle value of the X-axis of the right joystick
#define RCMID_RL 1450  // 右摇杆X轴中间值 (200-1000)   The middle value of the X-axis of the right joystick
#define RCMID_RR 1550  // 右摇杆X轴中间值 (1000-1800) The middle value of the X-axis of the right joystick

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

// 500hz PWM波频率 根据具体的驱动芯片设置合适的频率值
// PWM frequency, Set the appropriate frequency value according to the specific driver chip
#define freq_PWM 500
#define resolution_PWM 8  // 2的8次方  与arduino保持一致 2 to the 8th power is consistent with arduino

#define MAXSPEEDL 255  //摇杆控制左边电机速度，最大255
#define MAXSPEEDR 255  //摇杆控制右边电机速度，最大255

void setup() {
  Serial.begin(115200);  // 波特率 Baud rate

  pinMode(CH1_PIN, INPUT);  // Set our input pins as such
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

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
  Stop();
}

void loop() {
  ch1 = pulseIn(CH1_PIN, HIGH, 25000);
  ch2 = pulseIn(CH2_PIN, HIGH, 25000);
  ch3 = pulseIn(CH3_PIN, HIGH, 25000);
  ch4 = pulseIn(CH4_PIN, HIGH, 25000);
  ch5 = pulseIn(CH5_PIN, HIGH, 25000);
  ch6 = pulseIn(CH6_PIN, HIGH, 25000);

  if (ch1 <= RCMIN) ch1 = RCMIN;
  if (ch2 <= RCMIN) ch2 = RCMIN;
  if (ch3 <= RCMIN) ch3 = RCMIN;
  if (ch4 <= RCMIN) ch4 = RCMIN;
  if (ch5 <= RCMIN) ch5 = RCMIN;
  if (ch6 <= RCMIN) ch6 = RCMIN;
  if (ch1 >= RCMAX) ch1 = RCMAX;
  if (ch2 >= RCMAX) ch2 = RCMAX;
  if (ch3 >= RCMAX) ch3 = RCMAX;
  if (ch4 >= RCMAX) ch4 = RCMAX;
  if (ch5 >= RCMAX) ch5 = RCMAX;
  if (ch6 >= RCMAX) ch6 = RCMAX;

  // 松开摇杆后电机驱动停止使能，中值1500，防止漂移加50范围，左摇杆前后控制前进后退，左右控制原地转向。
  if (ch2 <= RCMID_LB && ch2 >= RCMID_LF && ch4 >= RCMID_LL && ch4 <= RCMID_LR) {
    Stop();

  } else {
    // 左侧摇杆Y轴控制小车前进后退，速度值根据摇杆值比例变化
    // The left joystick Y-axis controls the car forward and backward, and the speed value changes in proportion to the joystick value.
    if (ch2 >= RCMIN && ch2 <= RCMID_LF) {  // 前进 Forward

      // digitalWrite(15, HIGH);

      ledcWrite(PWM1B, LOW);
      ledcWrite(PWM4B, LOW);
      ledcWrite(PWM1A, map(ch2, RCMIN, RCMID_LF, MAXSPEEDR, 0));
      ledcWrite(PWM4A, map(ch2, RCMIN, RCMID_LF, MAXSPEEDR, 0));

      ledcWrite(PWM2A, LOW);
      ledcWrite(PWM3A, LOW);
      ledcWrite(PWM2B, map(ch2, RCMIN, RCMID_LF, MAXSPEEDL, 0));
      ledcWrite(PWM3B, map(ch2, RCMIN, RCMID_LF, MAXSPEEDL, 0));
      // Serial.println("FORWARD");
    } else if (ch2 >= RCMID_LB && ch2 <= RCMAX) {  // 后退 Back
      ledcWrite(PWM1A, LOW);
      ledcWrite(PWM4A, LOW);
      ledcWrite(PWM1B, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDR));
      ledcWrite(PWM4B, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDR));

      ledcWrite(PWM2B, LOW);
      ledcWrite(PWM3B, LOW);
      ledcWrite(PWM2A, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDL));
      ledcWrite(PWM3A, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDL));
      // Serial.println("BACK");
    } else if (ch4 >= RCMIN && ch4 <= RCMID_LL) {  // 左转 Left
      ledcWrite(PWM1B, LOW);
      ledcWrite(PWM4B, LOW);
      ledcWrite(PWM1A, map(ch4, RCMIN, RCMID_LL, MAXSPEEDR, 0));
      ledcWrite(PWM4A, map(ch4, RCMIN, RCMID_LL, MAXSPEEDR, 0));

      ledcWrite(PWM2B, LOW);
      ledcWrite(PWM3B, LOW);
      ledcWrite(PWM2A, map(ch4, RCMIN, RCMID_LL, MAXSPEEDL, 0));
      ledcWrite(PWM3A, map(ch4, RCMIN, RCMID_LL, MAXSPEEDL, 0));
      // Serial.println("LEFT");
    } else if (ch4 >= RCMID_LR && ch4 <= RCMAX) {  // 右转 Right
      ledcWrite(PWM1A, LOW);
      ledcWrite(PWM4A, LOW);
      ledcWrite(PWM1B, map(ch4, RCMID_LR, RCMAX, 0, MAXSPEEDR));
      ledcWrite(PWM1B, map(ch4, RCMID_LR, RCMAX, 0, MAXSPEEDR));

      ledcWrite(PWM2A, LOW);
      ledcWrite(PWM3A, LOW);
      ledcWrite(PWM2B, map(ch4, RCMAX, RCMID_LR, MAXSPEEDL, 0));
      ledcWrite(PWM3B, map(ch4, RCMAX, RCMID_LR, MAXSPEEDL, 0));
      // Serial.println("RIGHT");
    }
  }

  // 左摇杆前和右摇杆左控制前进状态差速转弯
  // The left joystick forward and the right joystick left control the forward state and differential turning.
  if (ch2 >= RCMIN && ch2 <= RCMID_LF) {
    if (ch1 >= RCMIN && ch1 <= RCMID_RL) {
      ledcWrite(PWM1B, LOW);
      ledcWrite(PWM4B, LOW);
      ledcWrite(PWM1A, map(ch2, RCMIN, RCMID_LF, MAXSPEEDR, 0));
      ledcWrite(PWM4A, map(ch2, RCMIN, RCMID_LF, MAXSPEEDR, 0));

      ledcWrite(PWM2A, LOW);
      ledcWrite(PWM3A, LOW);
      ledcWrite(PWM2B, map(ch1, RCMIN, RCMID_RL, 0, map(ch2, RCMIN, RCMID_LF, MAXSPEEDL, 0)));
      ledcWrite(PWM3B, map(ch1, RCMIN, RCMID_RL, 0, map(ch2, RCMIN, RCMID_LF, MAXSPEEDL, 0)));
    }
    // 左摇杆前和右摇杆右控制前进状态差速转弯
    // The left joystick forward and the right joystick right control the forward state and differential turning.
    else if (ch1 >= RCMID_RR && ch1 <= RCMAX) {
      ledcWrite(PWM1B, LOW);
      ledcWrite(PWM4B, LOW);
      ledcWrite(PWM1A, map(ch1, RCMAX, RCMID_RR, 0, map(ch2, RCMAX, RCMID_LF, MAXSPEEDR, 0)));
      ledcWrite(PWM4A, map(ch1, RCMAX, RCMID_RR, 0, map(ch2, RCMAX, RCMID_LF, MAXSPEEDR, 0)));

      ledcWrite(PWM2A, LOW);
      ledcWrite(PWM3A, LOW);
      ledcWrite(PWM2B, map(ch2, RCMAX, RCMID_LF, MAXSPEEDL, 0));
      ledcWrite(PWM3B, map(ch2, RCMAX, RCMID_LF, MAXSPEEDL, 0));
    }
  } else if (ch2 >= RCMID_LB && ch2 <= RCMAX) {
    // 左摇杆后和右摇杆左控制前进状态差速转弯
    // Left stick rear and right stick left control forward status differential turning
    if (ch1 >= RCMIN && ch1 <= RCMID_RL) {
      ledcWrite(PWM1A, LOW);
      ledcWrite(PWM4A, LOW);
      ledcWrite(PWM1B, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDR));
      ledcWrite(PWM4B, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDR));

      ledcWrite(PWM2B, LOW);
      ledcWrite(PWM3B, LOW);
      ledcWrite(PWM2A, map(ch1, RCMIN, RCMID_RL, 0, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDL)));
      ledcWrite(PWM3A, map(ch1, RCMIN, RCMID_RL, 0, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDL)));
    }
    // 左摇杆后和右摇杆右控制前进状态差速转弯
    // Left stick rear and right stick right control forward status differential turning
    else if (ch1 >= RCMID_RR && ch1 <= RCMAX) {
      ledcWrite(PWM1A, LOW);
      ledcWrite(PWM4A, LOW);
      ledcWrite(PWM1B, map(ch1, RCMAX, RCMID_RR, 0, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDR)));
      ledcWrite(PWM4B, map(ch1, RCMAX, RCMID_RR, 0, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDR)));

      ledcWrite(PWM2B, LOW);
      ledcWrite(PWM3B, LOW);
      ledcWrite(PWM2A, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDL));
      ledcWrite(PWM3A, map(ch2, RCMID_LB, RCMAX, 0, MAXSPEEDL));
    }
  }

  delay(10);
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
