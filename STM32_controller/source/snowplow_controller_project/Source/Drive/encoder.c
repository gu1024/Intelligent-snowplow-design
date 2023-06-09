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
#include "encoder.h"

int g_Encoder_A_Now = 0;
int g_Encoder_B_Now = 0;

// ��ʱ��3ͨ��1ͨ��2���ӱ�����M1A M1B
void Encoder_Init_TIM3(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  RCC_APB2PeriphClockCmd(Hal_2A_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = Hal_2A_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(Hal_2A_PORT, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(Hal_2B_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = Hal_2B_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(Hal_2B_PORT, &GPIO_InitStructure);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                  // Ԥ��Ƶ��
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;      // �趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // ѡ��ʱ�ӷ�Ƶ������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  TIM_SetCounter(TIM3, 0);
  TIM3->CNT = 0x7fff;
  TIM_Cmd(TIM3, ENABLE);
}

// ��ʱ��4ͨ��1ͨ��2���ӱ�����M2A M2B
void Encoder_Init_TIM4(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  RCC_APB2PeriphClockCmd(Hal_2A_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = Hal_2A_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(Hal_2A_PORT, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(Hal_2B_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = Hal_2B_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(Hal_2B_PORT, &GPIO_InitStructure);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                  // Ԥ��Ƶ��
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;      // �趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // ѡ��ʱ�ӷ�Ƶ������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);                       // ���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_SetCounter(TIM4, 0);
  TIM4->CNT = 0x7fff;
  TIM_Cmd(TIM4, ENABLE);
}

// ��ȡ�����������ֵ
s16 getTIMx_DetaCnt(TIM_TypeDef *TIMx)
{
  s16 cnt;
  cnt = 0x7fff - TIMx->CNT;
  TIMx->CNT = 0x7fff;
  return cnt;
}

// ��λʱ���ȡ����������
s16 Encoder_Read_CNT(u8 Encoder_id)
{
  s16 Encoder_TIM = 0;

  switch(Encoder_id) {
  case ENCODER_ID_A:
  {
    Encoder_TIM = 0x7fff - (short)TIM3 -> CNT; 
    TIM3 -> CNT = 0x7fff; 
    break;
  }

  case ENCODER_ID_B:
  {
    Encoder_TIM = 0x7fff - (short)TIM4 -> CNT; 
    TIM4 -> CNT = 0x7fff; 
    break;
  }

  default:  
    break;
  }

  return Encoder_TIM;
}

// �����ϵ�����ı������ܼ���
int Encoder_Get_Count_Now(u8 Encoder_id)
{
  if (Encoder_id == ENCODER_ID_A) 
    return g_Encoder_A_Now;

  if (Encoder_id == ENCODER_ID_B) 
    return g_Encoder_B_Now;

  return 0;
}

// ���±���������ֵ
void Encoder_Update_Count(u8 Encoder_id)
{
  switch (Encoder_id) {
  case ENCODER_ID_A:
  {
    g_Encoder_A_Now -= Encoder_Read_CNT(ENCODER_ID_A);
    break;
  }

  case ENCODER_ID_B:
  {
    g_Encoder_B_Now += Encoder_Read_CNT(ENCODER_ID_B);
    break;
  }

  default:
    break;
  }
}

void Encoder_Init(void)
{
  Encoder_Init_TIM3();    
  Encoder_Init_TIM4();     
}
