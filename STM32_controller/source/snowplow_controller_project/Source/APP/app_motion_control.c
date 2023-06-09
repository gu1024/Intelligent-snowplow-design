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

// С��ɲ����ʱʱ�䣬��λΪ10ms
#define MAX_STOP_COUNT       3

// �����ֵ��PWM����
int motorLeft = 0;
int motorRight = 0;

// �������ٶ�
int leftSpeedNow = 0;
int rightSpeedNow = 0;

// �������ٶ�����
int leftSpeedSet = 0;
int rightSpeedSet = 0;

// ������10msǰ������
int leftWheelEncoderNow = 0;
int rightWheelEncoderNow = 0;
int leftWheelEncoderLast = 0;
int rightWheelEncoderLast = 0;

// ����ֹͣʱ�䣬��ʱ�Զ��ر�ɲ������
u32 g_stop_count = 0;

s16 g_car_abandon = 0;

// TIM1ÿ10ms����һ���ж�
void TIM1_UP_IRQHandler(void)
{
  // ����Ƿ����ж��¼�
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
    if (Timer_Get_Count(COUNT_BEAT_ID))
      Timer_Count_Auto_Reduce(COUNT_BEAT_ID);

    #if ENABLE_MOTION_CONTROL
    Motion_Control_10ms();
    #endif
    
    // ����жϱ�־λ
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  
  }
}

// ʵʱ����С�������ٶ�
void Motion_Control_10ms(void)
{
  // ��ȡ�����ֵ�ǰʵ���ٶ�
  Get_Motor_Speed(&leftSpeedNow, &rightSpeedNow);

  if (leftSpeedSet || rightSpeedSet) {
    // Ŀ���ٶ�
    pid_Task_Left.speedSet = leftSpeedSet;
    pid_Task_Right.speedSet = rightSpeedSet;
    // ʵ���ٶ�
    pid_Task_Left.speedNow = leftSpeedNow;
    pid_Task_Right.speedNow = rightSpeedNow;

    // ִ��PID����
    Pid_Ctrl(&motorLeft, &motorRight, g_attitude.yaw);

    // ����PWM
    Motion_Set_PWM(motorLeft, motorRight);

    g_stop_count = 0;
  } else {
    PID_Reset_Yaw(g_attitude.yaw);

    if (g_stop_count < MAX_STOP_COUNT + 10) 
      g_stop_count++;
    
    // �ر�С��ɲ������
    if (g_stop_count == MAX_STOP_COUNT)
      Motor_Close_Brake();
  }
}

void Motion_Set_PWM(int motor_Left, int motor_Right)
{
  Motor_Set_Pwm(MOTOR_ID_1, motor_Left);
  Motor_Set_Pwm(MOTOR_ID_2, motor_Right);
}

// ������������
static double lSpd_mm_s = 0;
static double rSpd_mm_s = 0;

// �������������������
// ���תһȦ�������13�����壬1:45���ٱȣ���������תһȦ������(45*13*4) 2340������
// AB����������ź���λ��Ϊ90�㣬�ɼ����ת������
#define ENCODER_CNT_PER_ROUND       (2340)
#define WHEEL_CIRCUMFERENCE_CM      (21.36283)
#define ENCODER_CNT_10MS_2_SPD_MM_S (100.0 * WHEEL_CIRCUMFERENCE_CM * 10 / 2340.0)

void Get_Motor_Speed(int *leftSpeed, int *rightSpeed)
{
  Encoder_Update_Count(ENCODER_ID_A);
  leftWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_A);
  Encoder_Update_Count(ENCODER_ID_B);
  rightWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_B);
  
  // 10ms����
  *leftSpeed = (leftWheelEncoderNow - leftWheelEncoderLast) * WHEEL_CIRCUMFERENCE_CM;
  *rightSpeed = (rightWheelEncoderNow - rightWheelEncoderLast) * WHEEL_CIRCUMFERENCE_CM;
  lSpd_mm_s = (double)(leftWheelEncoderNow - leftWheelEncoderLast) * ENCODER_CNT_10MS_2_SPD_MM_S;
  rSpd_mm_s = (double)(rightWheelEncoderNow - rightWheelEncoderLast)* ENCODER_CNT_10MS_2_SPD_MM_S;

  // ��¼��һ���ڵı���������
  leftWheelEncoderLast = leftWheelEncoderNow;
  rightWheelEncoderLast = rightWheelEncoderNow;
}

// �ϱ�����ٶ�
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

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < MotionLEN - 1; i++)
    checknum += data_buffer[i];

  data_buffer[MotionLEN - 1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // ֡ͷ
  UART1_Put_Char(0x02); // ��ʶλ
  UART1_Put_Char(0x06); // ����λ����(�ֽ���)
  
  UART1_Put_Char(data_buffer[0]);
  UART1_Put_Char(data_buffer[1]);
  UART1_Put_Char(data_buffer[2]);
  UART1_Put_Char(data_buffer[3]);
  UART1_Put_Char(data_buffer[4]);
  UART1_Put_Char(data_buffer[5]);
  UART1_Put_Char(data_buffer[6]);
  
  UART1_Put_Char(0xBB); // ֡β
}

// PWM��������ֵ��Ӧ���ٶ�ԼΪ 1360mm/s
#define SPD_MM_S_MAX         (1360.0)
#define SPD_MM_S_2_PWM       (MOTOR_MAX_PULSE / SPD_MM_S_MAX)

// ����Ŀ���ٶ�
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
