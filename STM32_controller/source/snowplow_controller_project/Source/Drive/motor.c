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
#include "motor.h"
#include "Main.h"

#define PWM1 TIM2->CCR1
#define PWM2 TIM2->CCR2
#define PWM3 TIM2->CCR3
#define PWM4 TIM2->CCR4

// �Զ������ֵ����
static int myabs(int a)
{
  int temp;
  if (a < 0)
    temp = -a;
  else
    temp = a;
  return temp;
}

// ��ʼ���������
void MOTOR_GPIO_Init(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // ��������ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  // ѡ��Ҫ���Ƶ�GPIO����
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // �������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// �ر�С����ɲ�����ܣ��ó��ֲ��ٱ�������
void Motor_Close_Brake(void)
{
  PWM1 = MOTOR_MAX_PULSE;
  PWM2 = MOTOR_MAX_PULSE;
  PWM3 = MOTOR_MAX_PULSE;
  PWM4 = MOTOR_MAX_PULSE;
}

// ���PWM�ڳ�ʼ��, arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ��
void Motor_PWM_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  // ���½�Timer����Ϊȱʡֵ
  TIM_DeInit(TIM2);
  
  // ���ü��������С��ÿ��xxx�����Ͳ���һ�������¼� 
  TIM_TimeBaseStructure.TIM_Period = arr - 1 ;
  // Ԥ��Ƶϵ��Ϊ0����������Ԥ��Ƶ����ʱTIMER��Ƶ��Ϊ72MHzre.TIM_Prescaler =0;
  TIM_TimeBaseStructure.TIM_Prescaler = psc;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;    // ����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  // ����ȱʡֵ
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              // ����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = 0;       // ��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;       // ����ʱ������ֵС������ֵʱΪ�͵�ƽ
  TIM_OC1Init(TIM2, &TIM_OCInitStructure); //ʹ��ͨ��1
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // ����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = 0;       // ��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      // ����ʱ������ֵС������ֵʱΪ�͵�ƽ
  TIM_OC2Init(TIM2, &TIM_OCInitStructure); // ʹ��ͨ��2
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // ����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = 0;       //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      // ����ʱ������ֵС������ֵʱΪ�͵�ƽ
  TIM_OC3Init(TIM2, &TIM_OCInitStructure); //ʹ��ͨ��3
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // ����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = 0;       // ��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      // ����ʱ������ֵС������ֵʱΪ�͵�ƽ
  TIM_OC4Init(TIM2, &TIM_OCInitStructure); // ʹ��ͨ��4
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);      // ʹ��TIM3���ؼĴ���ARR

  TIM_Cmd(TIM2, ENABLE);                   //ʹ�ܶ�ʱ��2  
}

void Motor_m1_pwm(int speed)
{
  if (speed >= 0) {
    PWM1 = 0;
    PWM2 = speed;
  } else {
    PWM1 = myabs(speed);
    PWM2 = 0;
  }
}

void Motor_m2_pwm(int speed)
{
  if (speed >= 0) {
    PWM3 = speed;
    PWM4 = 0;
  } else {
    PWM3 = 0;
    PWM4 = myabs(speed);
  }
}

// ���õ���ٶȣ�speed:��3600, 0Ϊֹͣ
void Motor_Set_Pwm(u8 id, int speed)
{
  // ��������
  if (speed > MOTOR_MAX_PULSE) speed = MOTOR_MAX_PULSE;
  if (speed < -MOTOR_MAX_PULSE) speed = -MOTOR_MAX_PULSE;

  switch (id) {
  case MOTOR_ID_1:
  {
    Motor_m1_pwm(speed);
    break;
  }
  
  case MOTOR_ID_2:
  {
    Motor_m2_pwm(speed);
    break;
  }
  
  default:
    break;
  }
}


