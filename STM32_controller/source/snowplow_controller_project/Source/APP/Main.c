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
#include <string.h>
#include <stdio.h>
#include "Main.h"
#include "UART1.h"
#include "UART3.h"
#include "delay.h"
#include "JY901.h"
#include "DIO.h"
#include "adc.h"

int main(void)
{
  // ����ʱ��Ƶ��
  SysTick_init(72, 10);
  // ��ʼ������3�����ڶ�ȡ����������
  UART3_Init(9600);
  // ��ʼ������1�����ں�����ͨ��
  UART1_Init(115200);
  // �ȴ������ǳ�ʼ�����
	jy901_init();
  // ע��delay_ms(ms)�У�ms����<=1864
  Delay_Ms(1000); Delay_Ms(1000);
  
  // ��ʼ��ADC�����ڶ�ȡ��ص�ѹ
  Adc_Init();
  // ��ʼ��GPIO�����ڿ���LED�ͷ�������
  GPIO_Config();
  // ��ʼ��������ƽӿ�(PWM)
  MOTOR_GPIO_Init();
  // ���ò���Ƶ��PWMƵ�� 72000000/3600=20khz
  Motor_PWM_Init(MOTOR_MAX_PULSE, MOTOR_FREQ_DIVIDE);
  // ��ʼ������������ӿ�
  Encoder_Init();
  // ��ʼ����������ʱ��
  TIM1_Init();
  // ��ʼ��PID����
  PID_Init();
  Delay_Ms(1000);
  
  // printf("\n\nFirmware Version: V%d.%d\n", VERSION_MAJOR, VERSION_MINOR);
  
  int Call_10ms = 1;
  while (1) {
    // ���ա���������Ӧ�������͵Ĵ���ָ��
    if (Is_Recv_New_Cmd()) {
      Parse_Cmd_Data(Get_RxBuffer(), Get_CMD_Length());
      Clear_CMD_Flag();
    }
    
    // �������ϱ����ݸ�����
    if (Timer_Get_Count(COUNT_BEAT_ID) == 0) {
      // ����ص�ѹ�Ƿ����
      Bat_Update_Power_State();
      // ��ѹ���ͣ�����������
      if (Bat_Is_Low_Power())
        BUZZER_ON();

      // �ϱ����ڣ�40����
      if ((Call_10ms % 4 + 1) == 1) {
        Motion_Send_Data(); // ����ٶ�
        Acc_Send_Data();    // ������ ���ٶ�
        Gyro_Send_Data();   // ������ ���ٶ�
        Angle_Send_Data();  // ������ �Ƕ�
        Sensor_Send_Data(); // ��ص���
      }
      
      Call_10ms++;
      if (Call_10ms >= 8)
        Call_10ms = 0;

      Timer_Set_Count(COUNT_BEAT_ID, 1);
    }
  }
}
