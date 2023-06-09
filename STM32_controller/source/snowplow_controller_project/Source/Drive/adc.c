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
#include "adc.h"
#include "delay.h"
#include "stm32f10x_adc.h"
#include "Main.h"
#include "UART1.h"

// ��ص͵�ѹ��������ֵ����50��������ӳ�ʱ�䣬��λΪ����
// ���磺10*100=1000����1��
#define BAT_CHECK_COUNT         (100)
#define BAT_LOW_POWER_THRESHOLD (9.6f) // ����9.6V����Ϊ��ص�������

u8 g_bat_state = 1;          // ��ص͵�ѹ״̬����⵽�͵�ѹ��Ϊ0��ֻ��ͨ����λ�ָ�1
int Voltage_Z100 = 0;        // ��ص�ѹֵ
int Voltage_Low_Count = 0;   // �͵�ѹ����

// ��ʼ��ADC�����Թ���ͨ��Ϊ����Ĭ�Ͻ�����ͨ��0~3                                     
void Adc_Init(void)
{   
  ADC_InitTypeDef ADC_InitStructure; 
  GPIO_InitTypeDef GPIO_InitStructure;

  // ʹ��ADC1ͨ��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
 
  // ����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);

  // PA4 ��Ϊģ��ͨ����������       
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // ģ����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  ADC_DeInit(ADC1);  // ��λADC1 

  // ADC����ģʽ: ADC1��ADC2�����ڶ���ģʽ
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  // ģ��ת�������ڵ�ͨ��ģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  // ģ��ת�������ڵ���ת��ģʽ
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  // ת���������������ⲿ��������
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  // ADC�����Ҷ���
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  // ˳����й���ת����ADCͨ������Ŀ
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  // ����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���
  ADC_Init(ADC1, &ADC_InitStructure);

  // ʹ��ָ����ADC1
  ADC_Cmd(ADC1, ENABLE);
  
  // ʹ�ܸ�λУ׼
  ADC_ResetCalibration(ADC1);
   
  // �ȴ���λУ׼����
  while (ADC_GetResetCalibrationStatus(ADC1));
  
  // ����ADУ׼
  ADC_StartCalibration(ADC1);
 
  // �ȴ�У׼����
  while (ADC_GetCalibrationStatus(ADC1));
}

// ���ADCֵ��ch:ͨ��ֵ 0~3
u16 Get_Adc(u8 ch)   
{
  // ����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
  // ADC1,ADCͨ��,����ʱ��Ϊ239.5����
  ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );              
  // ʹ��ָ����ADC1������ת����������
  ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
  // �ȴ�ת������
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
  // �������һ��ADC1�������ת�����
  return ADC_GetConversionValue(ADC1);
}

// ��� ADC ��β���ƽ��ֵ, ch:ͨ��ֵ ; times:��������
u16 Adc_Get_Average(u8 ch, u8 times)
{
  u32 temp_val = 0;
  u8 t;
  for (t = 0; t < times; t++)
    temp_val += Get_Adc(ch);
  return temp_val / times;
}

// ��ò��ԭʼ��ѹֵ
float Adc_Get_Measure_Volotage(void)
{
  u16 adcx;
  float temp;
  // ADC Channel 4
  adcx = Adc_Get_Average(4, 5);
  temp = (float)adcx * (3.30f / 4096);
  return temp;
}

// ���ʵ�ʵ�ط�ѹǰ��ѹ
float Adc_Get_Battery_Volotage(void)
{
  float temp;
  temp = Adc_Get_Measure_Volotage();
  // ʵ�ʲ���ֵ�ȼ���ֵ��һ���
  temp = temp * 5.00f;
  return temp;
}

// ��ѯ��ص�ѹ״̬�����������������9.6V����0������9.6V����1
u8 Bat_Update_Power_State(void)
{
  if (g_bat_state) {
    Voltage_Z100 = (int)(Adc_Get_Battery_Volotage() * 100);
    if (Voltage_Z100 < (BAT_LOW_POWER_THRESHOLD * 100)) {
      Voltage_Low_Count++;
      if (Voltage_Low_Count > BAT_CHECK_COUNT)
        g_bat_state = 0;
    } else {
      Voltage_Low_Count = 0;
    }
  }
  return g_bat_state;
}

// �����幩���Ƿ���������������1����ѹ���ͷ���0
u8 Bat_Is_Low_Power(void)
{
  return (g_bat_state == 0);
}

// �ϱ���ص���
void Sensor_Send_Data(void)
{
  #define SensorLEN        7
  uint8_t data_buffer[SensorLEN] = {0};
  uint8_t i, checknum = 0;
  int bat = 0;
  
  bat = (int)(Adc_Get_Battery_Volotage() * 100); // V
  data_buffer[0] = (int) (bat / 100);
  data_buffer[1] = (int) (bat % 100);
  data_buffer[2] = 0;
  data_buffer[3] = 0;
  data_buffer[4] = 0;
  data_buffer[5] = 0;

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < SensorLEN - 1; i++)
    checknum += data_buffer[i];
  data_buffer[SensorLEN - 1] = checknum & 0xFF;

  UART1_Put_Char(0x55); // ֡ͷ
  UART1_Put_Char(0x06); // ��ʶλ
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
