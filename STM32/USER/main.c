/*
 * main function
 * author gunengyu
 * email 0xnoam1024@gmail.com
 * Copyright (C) 1960-2023  Northeast Petroleum University
 * All Rights Reserved.
 */
#include "stm32f10x.h"
#include "delay.h"
#include "gpio.h"
#include "moto.h"
#include "pwm.h"
#include "adc.h"
#include "usart.h"
#include "encoder.h"

int main(void)
 {	
	 u16 encoder_A,encoder_B;
	 int Velocity_PWM1,Velocity_PWM2;
	 u16 adcx;
	 float vcc;
   SystemInit(); //����ϵͳʱ��Ϊ72M   
   delay_init();    //��ʱ������ʼ��
   uart_init(9600);		//���ڳ�ʼ��
	 
   adc_Init();				//ADC1�ĳ�ʼ��   
	 
   PWM_Int(7199,0);      //��ʼ��pwm��� 72000 000 /7199+1=10000 
   Encoder_Init_Tim2();
   Encoder_Init_Tim4();
  while(1)
	{
//	    moto(0);                //moto=0ʱ��ת
//	    moto(1);                //moto=1ʱ��ת

		adcx=Get_adc_Average(ADC_Channel_2,10);  //��ȡadc��ֵ
		vcc=(float)adcx*(3.3*11/4096);     		 //��ǰ��ѹ
		encoder_A=Read_Encoder(2);
		encoder_B=Read_Encoder(4);               //��ȡ��������ֵ
		Velocity_PWM1=Velocity_A(encoder_A);
		Velocity_PWM2=Velocity_B(encoder_B);
		Set_PWM(Velocity_PWM1,Velocity_PWM2);
		printf("��ǰ��ѹ=%6.2f V  Encoder_A = %d  Encoder_B=%d\r\n",vcc,encoder_A,encoder_B);				//��ӡ��ǰ��ѹ������С�������λ	
	}
 }

