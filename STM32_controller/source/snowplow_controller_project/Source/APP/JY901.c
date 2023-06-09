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
#include "JY901.h"
#include "UART3.h"
#include "UART1.h"
#include "Main.h"
#include "delay.h"
#include "DIO.h"

struct STime    stcTime;
struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;
struct SMag     stcMag;
struct SDStatus stcDStatus;
struct SPress   stcPress;
struct SLonLat  stcLonLat;
struct SGPSV    stcGPSV;
struct SQ       stcQ;

char JY_CMD_UNLOCK[5] = {0xFF,0xAA,0x69,0x88,0xB5};        	 // ����
char JY_CMD_ALGORITHM[5] = {0xFF,0xAA,0x24,0x01,0x00};       // �����㷨Ϊ����
char JY_CMD_ACC_CALIBRATION[5] = {0xFF,0xAA,0x01,0x01,0x00}; // ִ�м��ٶ�У׼ģʽ
char JY_CMD_YAW_ZERO[5] = {0xFF,0xAA,0x01,0x04,0x00}; 			 // ִ�к��������
char JY_CMD_SAVE_CFG[5] = {0xFF,0xAA,0x00,0x00,0x00};        // ���浱ǰ����

// ͨ������3�������Ƿ���ָ��
void sendcmd(char cmd[])
{
  for (char i = 0; i < 5; i++)
    UART3_Put_Char(cmd[i]);
}

void jy901_init(void)
{
	sendcmd(JY_CMD_UNLOCK);
  // ����
  Delay_Ms(100);
	
  sendcmd(JY_CMD_ALGORITHM);
  // �ȴ������㷨
  Delay_Ms(100);
	
	sendcmd(JY_CMD_SAVE_CFG);
  Delay_Ms(100);
	//���浱ǰ����
}

void jy901_calibration(void)
{
	sendcmd(JY_CMD_UNLOCK);
  // ����
  Delay_Ms(100);
	
  sendcmd(JY_CMD_ACC_CALIBRATION);
  // �ȴ�ģ��У׼���
  Delay_Ms(100);
	
	sendcmd(JY_CMD_YAW_ZERO);
  // �ȴ����������
  Delay_Ms(100);

  sendcmd(JY_CMD_SAVE_CFG);
  Delay_Ms(100);

}

// ����3���ݴ�������������ÿ�յ�һ�����ݣ�����һ��
void CopeSerial3Data(unsigned char ucData)
{
  static unsigned char ucRxBuffer[250];
  static unsigned char ucRxCnt = 0;  
  
  // ���յ������ݴ��뻺������
  ucRxBuffer[ucRxCnt++] = ucData;

  // ����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
  if (ucRxBuffer[0] != 0x55) {
    ucRxCnt=0;
    return;
  }

  // ���ݲ���11�����򷵻�
  if (ucRxCnt < 11)
    return;

  // �ж���������������
  // ����������Ҫͨ��ά�عٷ��ṩ����λ�����ã���������������ǲŻᷢ�͸������ݰ�
  // ����Ŀ����ע ���ٶȡ����ٶȡ��Ƕ� ��������
  switch(ucRxBuffer[1]) {
    case 0x50:  memcpy(&stcTime, &ucRxBuffer[2], 8);   break;
    case 0x51:  memcpy(&stcAcc, &ucRxBuffer[2], 8);    break; // ���ٶ�
    case 0x52:  memcpy(&stcGyro, &ucRxBuffer[2], 8);   break; // ���ٶ�
    case 0x53:  memcpy(&stcAngle, &ucRxBuffer[2], 8);  break; // �Ƕ�
    case 0x54:  memcpy(&stcMag, &ucRxBuffer[2], 8);    break;
    case 0x55:  memcpy(&stcDStatus, &ucRxBuffer[2], 8);break;
    case 0x56:  memcpy(&stcPress, &ucRxBuffer[2], 8);  break;
    case 0x57:  memcpy(&stcLonLat, &ucRxBuffer[2], 8); break;
    case 0x58:  memcpy(&stcGPSV, &ucRxBuffer[2], 8);   break;
    case 0x59:  memcpy(&stcQ, &ucRxBuffer[2], 8);      break;
  }

  ucRxCnt = 0; // ��ջ�����
}

// �ϱ������Ǽ��ٶ�
void Acc_Send_Data(void)
{
  #define AccLEN        7
  uint8_t data_buffer[AccLEN] = {0};
  uint8_t i, checknum = 0;
  
  // ���ֽ���ǰ�����ֽ��ں�
  // X����ٶ�
  data_buffer[0] = stcAcc.a[0]&0xFF;
  data_buffer[1] = (stcAcc.a[0]>>8)&0xFF;
  // Y����ٶ�
  data_buffer[2] = stcAcc.a[1]&0xFF;
  data_buffer[3] = (stcAcc.a[1]>>8)&0xFF;
  // Z����ٶ�
  data_buffer[4] = stcAcc.a[2]&0xFF;
  data_buffer[5] = (stcAcc.a[2]>>8)&0xFF;

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < AccLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[AccLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // ֡ͷ
  UART1_Put_Char(0x03); // ��ʶλ
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

// �ϱ������ǽ��ٶ�
void Gyro_Send_Data(void)
{
  #define GyroLEN        7
  uint8_t data_buffer[GyroLEN] = {0};
  uint8_t i, checknum = 0;
  
  // ���ֽ���ǰ�����ֽ��ں�
  // X����ٶ�
  data_buffer[0] = stcGyro.w[0]&0xFF;
  data_buffer[1] = (stcGyro.w[0]>>8)&0xFF;
  // Y����ٶ�
  data_buffer[2] = stcGyro.w[1]&0xFF;
  data_buffer[3] = (stcGyro.w[1]>>8)&0xFF;
  // Z����ٶ�
  data_buffer[4] = stcGyro.w[2]&0xFF;
  data_buffer[5] = (stcGyro.w[2]>>8)&0xFF;

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < GyroLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[GyroLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // ֡ͷ
  UART1_Put_Char(0x04); // ��ʶλ
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

// �ϱ�������ŷ����
void Angle_Send_Data(void)
{
  #define AngleLEN        7
  uint8_t data_buffer[AngleLEN] = {0};
  uint8_t i, checknum = 0;
  
  // ���ֽ���ǰ�����ֽ��ں�
  // Roll
  data_buffer[0] = stcAngle.Angle[0]&0xFF;
  data_buffer[1] = (stcAngle.Angle[0]>>8)&0xFF;
  // Pitch
  data_buffer[2] = stcAngle.Angle[1]&0xFF;
  data_buffer[3] = (stcAngle.Angle[1]>>8)&0xFF;
  // Yaw
  data_buffer[4] = stcAngle.Angle[2]&0xFF;
  data_buffer[5] = (stcAngle.Angle[2]>>8)&0xFF;

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < AngleLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[AngleLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // ֡ͷ
  UART1_Put_Char(0x05); // ��ʶλ
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
