/***********************************************************************
Copyright (c) 2022, Northeast Petroleum University

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

? ? http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/
#include "app_motion_control.h"
#include "Main.h"
#include "UART1.h"

#include <math.h>

// 小车刹车超时时间，单位为10ms
#define MAX_STOP_COUNT       3

// 左右轮电机PWM变量
int motorLeft = 0;
int motorRight = 0;

// 左右轮速度
int leftSpeedNow = 0;
int rightSpeedNow = 0;

// 左右轮速度设置
int leftSpeedSet = 0;
int rightSpeedSet = 0;

// 编码器10ms前后数据
int leftWheelEncoderNow = 0;
int rightWheelEncoderNow = 0;
int leftWheelEncoderLast = 0;
int rightWheelEncoderLast = 0;

// 计算停止时间，超时自动关闭刹车功能
u32 g_stop_count = 0;

s16 g_car_abandon = 0;

// TIM1每10ms产生一次中断
void TIM1_UP_IRQHandler(void)
{
  // 检查是否发生中断事件
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
    if (Timer_Get_Count(COUNT_BEAT_ID))
      Timer_Count_Auto_Reduce(COUNT_BEAT_ID);

    #if ENABLE_MOTION_CONTROL
    Motion_Control_10ms();
    #endif
    
    // 清除中断标志位
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  
  }
}

// 实时计算小车运行速度
void Motion_Control_10ms(void)
{
  // 获取左右轮当前实际速度
  Get_Motor_Speed(&leftSpeedNow, &rightSpeedNow);

  if (leftSpeedSet || rightSpeedSet) {
    // 目标速度
    pid_Task_Left.speedSet = leftSpeedSet;
    pid_Task_Right.speedSet = rightSpeedSet;
    // 实际速度
    pid_Task_Left.speedNow = leftSpeedNow;
    pid_Task_Right.speedNow = rightSpeedNow;

    // 执行PID控制
    Pid_Ctrl(&motorLeft, &motorRight, g_attitude.yaw);

    // 设置PWM
    Motion_Set_PWM(motorLeft, motorRight);

    g_stop_count = 0;
  } else {
    PID_Reset_Yaw(g_attitude.yaw);

    if (g_stop_count < MAX_STOP_COUNT + 10) 
      g_stop_count++;
    
    // 关闭小车刹车功能
    if (g_stop_count == MAX_STOP_COUNT)
      Motor_Close_Brake();
  }
}

void Motion_Set_PWM(int motor_Left, int motor_Right)
{
  Motor_Set_Pwm(MOTOR_ID_1, motor_Left);
  Motor_Set_Pwm(MOTOR_ID_2, motor_Right);
}

// 计算左右轮速
static double lSpd_mm_s = 0;
static double rSpd_mm_s = 0;

// 金属输出轴编码器电机：
// 电机转一圈单相输出13个脉冲，1:45减速比，电机输出轴转一圈最大输出(45*13*4) 2340个计数
// AB相输出脉冲信号相位差为90°，可检测电机转动方向
#define ENCODER_CNT_PER_ROUND       (2340)
#define WHEEL_CIRCUMFERENCE_CM      (21.36283)
#define ENCODER_CNT_10MS_2_SPD_MM_S (100.0 * WHEEL_CIRCUMFERENCE_CM * 10 / 2340.0)

void Get_Motor_Speed(int *leftSpeed, int *rightSpeed)
{
  Encoder_Update_Count(ENCODER_ID_A);
  leftWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_A);
  Encoder_Update_Count(ENCODER_ID_B);
  rightWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_B);
  
  // 10ms测速
  *leftSpeed = (leftWheelEncoderNow - leftWheelEncoderLast) * WHEEL_CIRCUMFERENCE_CM;
  *rightSpeed = (rightWheelEncoderNow - rightWheelEncoderLast) * WHEEL_CIRCUMFERENCE_CM;
  lSpd_mm_s = (double)(leftWheelEncoderNow - leftWheelEncoderLast) * ENCODER_CNT_10MS_2_SPD_MM_S;
  rSpd_mm_s = (double)(rightWheelEncoderNow - rightWheelEncoderLast)* ENCODER_CNT_10MS_2_SPD_MM_S;

  // 记录上一周期的编码器数据
  leftWheelEncoderLast = leftWheelEncoderNow;
  rightWheelEncoderLast = rightWheelEncoderNow;
}

// 上报电机速度
void Motion_Send_Data(void)
{
  #define MotionLEN        7
  uint8_t data_buffer[MotionLEN] = {0};
  uint8_t i, checknum = 0;
  
  if (lSpd_mm_s < 0) {
    data_buffer[0] = 0x00;
    uint16_t spd = (uint16_t)fabs(lSpd_mm_s);
    data_buffer[1] = spd&0xFF;
    data_buffer[2] = (spd>>8)&0xFF;
  } else {
    data_buffer[0] = 0xFF;
    uint16_t spd = (uint16_t)lSpd_mm_s;
    data_buffer[1] = spd&0xFF;
    data_buffer[2] = (spd>>8)&0xFF;
  }

  if (rSpd_mm_s < 0) {
    data_buffer[3] = 0x00;
    uint16_t spd = (uint16_t)fabs(rSpd_mm_s);
    data_buffer[4] = spd&0xFF;
    data_buffer[5] = (spd>>8)&0xFF;
  } else {
    data_buffer[3] = 0xFF;
    uint16_t spd = (uint16_t)rSpd_mm_s;
    data_buffer[4] = spd&0xFF;
    data_buffer[5] = (spd>>8)&0xFF;
  }

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < MotionLEN - 1; i++)
    checknum += data_buffer[i];

  data_buffer[MotionLEN - 1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // 帧头
  UART1_Put_Char(0x02); // 标识位
  UART1_Put_Char(0x06); // 数据位长度(字节数)
  
  UART1_Put_Char(data_buffer[0]);
  UART1_Put_Char(data_buffer[1]);
  UART1_Put_Char(data_buffer[2]);
  UART1_Put_Char(data_buffer[3]);
  UART1_Put_Char(data_buffer[4]);
  UART1_Put_Char(data_buffer[5]);
  UART1_Put_Char(data_buffer[6]);
  
  UART1_Put_Char(0xBB); // 帧尾
}

// PWM的最大输出值对应的速度约为 1360mm/s
#define SPD_MM_S_MAX         (1360.0)
#define SPD_MM_S_2_PWM       (MOTOR_MAX_PULSE / SPD_MM_S_MAX)

// 设置目标速度
void Motion_Test_SpeedSet(uint8_t index_l, int16_t left , 
                          uint8_t index_r, int16_t right)
{
  left = left * SPD_MM_S_2_PWM;
  right = right * SPD_MM_S_2_PWM;

  if (left > MOTOR_MAX_PULSE) 
    left = MOTOR_MAX_PULSE;
  if (right > MOTOR_MAX_PULSE) 
    right = MOTOR_MAX_PULSE;

  if (index_l == 0)
    leftSpeedSet = -left;
  else
    leftSpeedSet = left;
    
  if (index_r == 0)
    rightSpeedSet = -right;
  else
    rightSpeedSet = right;
}
