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
***********************************************************************/
#include "timer.h"
#include "Main.h"

#include <stdio.h>
#include "DIO.h"

// ϵͳ������ 
u16 Count_Beat = 0;
int temp = 0;

u16 Timer_Get_Count(u8 id)
{
  if (id == COUNT_BEAT_ID)
    return Count_Beat;
  return 65535;
}

void Timer_Set_Count(u8 id, u16 value)
{
  if (id == COUNT_BEAT_ID)
    Count_Beat = value;
}

void Timer_Count_Auto_Reduce(u8 id)
{
  if (id == COUNT_BEAT_ID)
    Count_Beat--;
}

// TIM1��ʼ������ʱ10����
void TIM1_Init(void)
{   
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // TIM1ʱ��ʹ��

  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  // ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ��������5000Ϊ500ms
  TIM_TimeBaseStructure.TIM_Period = 99;
  // ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ��10Khz�ļ���Ƶ��  
  TIM_TimeBaseStructure.TIM_Prescaler =7199;
  // ����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  // TIM���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  // ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  // TIM1�ж�  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ�3��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  // �����ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     // IRQͨ����ʹ��
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_Cmd(TIM1, ENABLE); 
}
