/*********************************************************************
  MotorDriver-IIC-AT8236 Test

  IIC芯片驱动电机测试程序，adafruit库文件端口直接驱动。
**********************************************************************/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int motorspeed = 3000;// Motor MAX Speed   limits（0-4096）

void setup() {
  Serial.begin(9600);
  Serial.println("MotorDriver-IIC Test");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  //  pwm.setPWMFreq(1500);  // This is the maximum PWM frequency
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  Wire.setClock(400000);
}

void loop() {
  for (int i = 0; i < 4095; i += 10) {
    pwm.setPWM(0, 0, i);  // M1顺时针
    pwm.setPWM(1, 4095, 0);
    pwm.setPWM(2, 0, i);  // M2顺时针
    pwm.setPWM(3, 4095, 0);
    pwm.setPWM(7, 0, i);  // M3顺时针
    pwm.setPWM(6, 4095, 0);
    pwm.setPWM(5, 0, i);  // M4顺时针
    pwm.setPWM(4, 4095, 0);
    delay(5);
  }

  for (int i = 4095; i >= 0; i -= 10) {
    pwm.setPWM(0, 0, i);  // M1顺时针
    pwm.setPWM(1, 4095, 0);
    pwm.setPWM(2, 0, i);  // M2顺时针
    pwm.setPWM(3, 4095, 0);
    pwm.setPWM(7, 0, i);  // M3顺时针
    pwm.setPWM(6, 4095, 0);
    pwm.setPWM(5, 0, i);  // M4顺时针
    pwm.setPWM(4, 4095, 0);
    delay(5);
  }

  pwm.setPWM(1, 0, motorspeed);  // M1逆时针
  pwm.setPWM(0, 4095, 0);
  pwm.setPWM(3, 0, motorspeed);  // M2逆时针
  pwm.setPWM(2, 4095, 0);
  pwm.setPWM(6, 0, motorspeed);  // M3逆时针
  pwm.setPWM(7, 4095, 0);
  pwm.setPWM(4, 0, motorspeed);  // M4逆时针
  pwm.setPWM(5, 4095, 0);
  delay(1000);

  pwm.setPWM(0, 0, 4095);  // 刹车
  pwm.setPWM(1, 0, 4095);
  pwm.setPWM(2, 0, 4095);  // 刹车
  pwm.setPWM(3, 0, 4095);
  pwm.setPWM(7, 0, 4095);  // 刹车
  pwm.setPWM(6, 0, 4095);
  pwm.setPWM(5, 0, 4095);  // 刹车
  pwm.setPWM(4, 0, 4095);
  delay(500);

  pwm.setPWM(0, 0, motorspeed);  // M1顺时针
  pwm.setPWM(1, 4095, 0);
  pwm.setPWM(2, 0, motorspeed);  // M2顺时针
  pwm.setPWM(3, 4095, 0);
  pwm.setPWM(7, 0, motorspeed);  // M3顺时针
  pwm.setPWM(6, 4095, 0);
  pwm.setPWM(5, 0, motorspeed);  // M4顺时针
  pwm.setPWM(4, 4095, 0);
  delay(1000);

  pwm.setPWM(0, 4095, 0);  // 自由停止
  pwm.setPWM(1, 4095, 0);
  pwm.setPWM(2, 4095, 0);  // 自由停止
  pwm.setPWM(3, 4095, 0);
  pwm.setPWM(7, 4095, 0);  // 自由停止
  pwm.setPWM(6, 4095, 0);
  pwm.setPWM(5, 4095, 0);  // 自由停止
  pwm.setPWM(4, 4095, 0);
  delay(500);

  for (int val = 90; val <= 480; val += 2) {
    pwm.setPWM(8, 0, val);   //90为0度位置
    pwm.setPWM(9, 0, val);   //90为0度位置
    pwm.setPWM(10, 0, val);  //90为0度位置
    pwm.setPWM(11, 0, val);  //90为0度位置
    pwm.setPWM(12, 0, val);  //90为0度位置
    pwm.setPWM(13, 0, val);  //90为0度位置
    pwm.setPWM(14, 0, val);  //90为0度位置
    pwm.setPWM(15, 0, val);  //90为0度位置
    delay(10);
  }

  for (int val = 480; val >= 90; val -= 2) {
    pwm.setPWM(8, 0, val);   //480为180度位置
    pwm.setPWM(9, 0, val);   //480为180度位置
    pwm.setPWM(10, 0, val);  //480为180度位置
    pwm.setPWM(11, 0, val);  //480为180度位置
    pwm.setPWM(12, 0, val);  //480为180度位置
    pwm.setPWM(13, 0, val);  //480为180度位置
    pwm.setPWM(14, 0, val);  //480为180度位置
    pwm.setPWM(15, 0, val);  //480为180度位置
    delay(10);
  }

  // pwm.writeMicroseconds(8, 500);  //0度位置
  // delay(1000);
  // pwm.writeMicroseconds(8, 2400);  //180度位置
  // delay(1000);
}
