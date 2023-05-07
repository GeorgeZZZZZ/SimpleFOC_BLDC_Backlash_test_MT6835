#include <Arduino.h>
#include <SimpleFOC.h>
#include <arduino-timer.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"

#define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin
MagneticSensorMT6835 sensor_0(SENSOR1_CS);

// 在线电流检测实例
// InlineCurrentSense current_sense_0 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense_0 = InlineCurrentSense(0.01f, 50.0f, 39, 36);
// InlineCurrentSense current_sense_0 = InlineCurrentSense(50.0, 39, 36);

// 电机参数
BLDCMotor motor_0 = BLDCMotor(11);
BLDCDriver3PWM driver_0 = BLDCDriver3PWM(32, 33, 25, 22);

// include commander interface
Commander command = Commander(Serial);
void doMotor_0(char *cmd)
{
  command.motor(&motor_0, cmd);
}

float tars[] = {1, 3.14, 6.28, 3.14};
const int tars_len = sizeof(tars) / sizeof(float);
float target = 0;
int tar_i = 0;

float tar_4_0_1_time = 1000;  // 1s
float tar_2_3_4_time = 5000; // 15s

auto timer_401 = timer_create_default(); // create a timer with default settings
auto timer_234 = timer_create_default(); // create a timer with default settings
bool motor_dir = false;

uint64_t n = 0;
bool timer_234_callback(void *)
{
  tar_i++;
  if (tar_i >= 4)
    tar_i = 0;
  target = tars[tar_i];
  Serial.println(target);
  return true;
}
bool timer_401_callback(void *)
{
  tar_i++;
  target = tars[tar_i];
  Serial.println(target);
  return true;
}

void setup()
{
  Serial.begin(921600);
  // Serial.begin(115200);
  Serial.println("");

  sensor_0.init();
  Serial.println("sensor ok");
  // 连接motor对象与传感器对象
  motor_0.linkSensor(&sensor_0);
  Serial.println("sensor linked to motor");
  //*
  // 供电电压设置 [V]
  driver_0.voltage_power_supply = 24;
  driver_0.init();
  Serial.println("driver ok");

  // 连接电机和driver对象
  motor_0.linkDriver(&driver_0);
  Serial.println("motor linked to driver");

  // 电流检测
  current_sense_0.init();
  current_sense_0.gain_b *= -1;
  current_sense_0.skip_align = true;
  motor_0.linkCurrentSense(&current_sense_0);

  // 控制环
  // 其他模式 TorqueControlType::voltage TorqueControlType::dc_current
  // max current senser range <3.3a
  motor_0.torque_controller = TorqueControlType::voltage;
  // motor_0.torque_controller = TorqueControlType::dc_current;

  // FOC模型选择
  motor_0.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // 运动控制模式设置
  motor_0.controller = MotionControlType::angle;
  // motor_0.controller = MotionControlType::torque;

  // motor_0.voltage_sensor_align = 6;

  // 电流限制
  motor_0.current_limit = 3.3; // current senser max 3.3, driver can hold max 4.4
  // 电压限制
  motor_0.voltage_limit = 24; // 24
  // rad/s
  // 6.28 for no gearbox
  motor_0.velocity_limit = 6.28; // 30; //0.5;//6.28; //= 115; //max 150
  // motor_0.velocity_limit = 150; //30; //0.5;//6.28; //= 115; //max 150

  //tars[0] = 0;
  // 3.09795942; //177.5
  // 0.13469389; //177.5 / 23 = 7.717391304
  // 71.2530667; //177.5 * 23 = 4082.5
  //tars[2] = 6.19591885; // 177.5 * 2 = 355
  tars[1] = tars[3] = -6.28318531; // 360
  //tars[2] = -148.789319; // 360 + 355 * 23 = 8525
  tars[2] = -12.4791042; //360 + 355 = 715

  tar_4_0_1_time = 10000;
  tar_2_3_4_time = 15000;

  // 23.04.10
  // ok for 115, no obvious overshoot, torque using voltage
  //
  //*
  motor_0.P_angle.P = 20;
  motor_0.P_angle.I = 0;
  motor_0.PID_velocity.P = 0.5;            // 0.5
  motor_0.PID_velocity.I = 15;             // 10
  motor_0.PID_velocity.output_ramp = 1000; // 1000
  motor_0.LPF_velocity.Tf = 0.001;
  motor_0.PID_current_q.P = 3;
  motor_0.PID_current_q.I = 300;
  motor_0.LPF_current_q.Tf = 0.001;
  //*/

  // 23.04.13
  // use to mesure torque, ok with no gearbox, torque using current
  // 2.4~2.47a at 300g
  /*
  motor_0.P_angle.P = 10;
  motor_0.P_angle.I = 0;
  motor_0.PID_velocity.P = 0.5;          //0.5
  motor_0.PID_velocity.I = 20;             //10
  motor_0.PID_velocity.output_ramp = 1000; //1000
  motor_0.LPF_velocity.Tf = 0.001;
  motor_0.PID_current_q.P = 3;
  motor_0.PID_current_q.I = 300;
  motor_0.LPF_current_q.Tf = 0.001;
  //*/

  // 初始化电机
  motor_0.init();
  Serial.println("motor ok");

  // 初始化 FOC
  motor_0.initFOC();
  Serial.println("foc ok");

  //*/
}

uint8_t backlash_check_status = 0;
void loop()
{

  motor_0.loopFOC();

  motor_0.move(target);
  Serial.println(target);
  switch (backlash_check_status)
  {
  case 0:
    timer_401.every(tar_4_0_1_time, timer_401_callback);
    backlash_check_status = 1;
    break;
  case 1:
    timer_401.tick();
    if (tar_i == 2)
    {
      timer_401.cancel();
      timer_234.every(tar_2_3_4_time, timer_234_callback);
      backlash_check_status = 2;
    }
    break;
  case 2:
    timer_234.tick();
    if (tar_i == 0)
    {
      timer_234.cancel();
      timer_401.every(tar_4_0_1_time, timer_401_callback);
      backlash_check_status = 1;
    }
    break;
  }
}
