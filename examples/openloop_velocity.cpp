// Deng's FOC 开环速度控制例程 测试库：SimpleFOC 2.2.1 测试硬件：灯哥开源FOC V3.0
// 串口中输入"T+数字"设定两个电机的转速，如设置电机以 10rad/s 转动，输入 "T10"，电机上电时默认会以 5rad/s 转动
// 在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(14) 中的值，设置为自己的极对数数字
// 程序默认设置的供电电压为 16.8V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
// changed to GM4827H

#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(11);  
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
  
// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1  = BLDCDriver3PWM(26,27,14,12);

//目标变量
float target_velocity = 5;

//串口指令设置
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  // put your setup code here, to run once:
  //should support 5mb baud rate, but dont know how to set
  //Serial.begin(2000000);
  //Serial.begin(921600);
  //Serial.println();
  //Serial.print("ready to start test!");



  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 5;   // [V] using 12v hot very fast in motor and foc, and foc making some current noise sound
  motor.velocity_limit = 40; // [rad/s]
  
  driver1.voltage_power_supply = 24;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 5;   // [V]
  motor1.velocity_limit = 40; // [rad/s]

 
  //开环控制模式设置
  motor.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  //初始化硬件
  motor.init();
  motor1.init();


  //增加 T 指令
  command.add('v', doTarget, "target velocity");

  Serial.begin(921600);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(5000);
  
}

int n = 0;
void loop() {
  motor.move(target_velocity);
  motor1.move(target_velocity);

  //用户通讯
  command.run();
}