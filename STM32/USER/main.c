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
   SystemInit(); //配置系统时钟为72M   
   delay_init();    //延时函数初始化
   uart_init(9600);		//串口初始化
	 
   adc_Init();				//ADC1的初始化   
	 
   PWM_Int(7199,0);      //初始化pwm输出 72000 000 /7199+1=10000 
   Encoder_Init_Tim2();
   Encoder_Init_Tim4();
  while(1)
	{
//	    moto(0);                //moto=0时正转
//	    moto(1);                //moto=1时反转

		adcx=Get_adc_Average(ADC_Channel_2,10);  //获取adc的值
		vcc=(float)adcx*(3.3*11/4096);     		 //求当前电压
		encoder_A=Read_Encoder(2);
		encoder_B=Read_Encoder(4);               //读取编码器数值
		Velocity_PWM1=Velocity_A(encoder_A);
		Velocity_PWM2=Velocity_B(encoder_B);
		Set_PWM(Velocity_PWM1,Velocity_PWM2);
		printf("当前电压=%6.2f V  Encoder_A = %d  Encoder_B=%d\r\n",vcc,encoder_A,encoder_B);				//打印当前电压，保留小数点后两位	
	}
 }

