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
#include "pid.h"
#include "Main.h"

attitude_t g_attitude;

struct pid_uint pid_Task_Left;
struct pid_uint pid_Task_Right;

PID pid_Yaw = {0, 0.4, 0, 0.1, 0, 0, 0};

int s_offset = 0;

// PID
float f_kp = 0.1;
float f_ki = 0.0;
float f_kd = 4.0;

void PID_Init(void)
{
  // ��1024ת��Ϊ�������㣬���⸡��������
  // �����ٶ�pid
  pid_Task_Left.Kp = 1024 * PID_KP_DEF_L;
  pid_Task_Left.Ki = 1024 * PID_KI_DEF_L;
  pid_Task_Left.Kd = 1024 * PID_KD_DEF_L;
  pid_Task_Left.Ur = 1024 * MOTOR_MAX_PULSE * 2;
  pid_Task_Left.Adjust = 0;
  pid_Task_Left.En = 1;
  pid_Task_Left.speedSet = 0;
  pid_Task_Left.speedNow = 0;
  reset_Uk(&pid_Task_Left);

  // �����ٶ�pid
  pid_Task_Right.Kp = 1024 * PID_KP_DEF_R;
  pid_Task_Right.Ki = 1024 * PID_KI_DEF_R;
  pid_Task_Right.Kd = 1024 * PID_KD_DEF_R;
  pid_Task_Right.Ur = 1024 * MOTOR_MAX_PULSE * 2;
  pid_Task_Right.Adjust = 0;
  pid_Task_Right.En = 1;
  pid_Task_Right.speedSet = 0;
  pid_Task_Right.speedNow = 0;
  reset_Uk(&pid_Task_Right);
}

// �ڳ�ʼ��ʱ���ã��ı�PID����ʱ�п�����Ҫ����
void reset_Uk(struct pid_uint *p)
{
  p->U_kk = 0;
  p->ekk = 0;
  p->ekkk = 0;
}

void reset_PID(struct pid_uint *p)
{
  p->U_kk = 0;
  p->ekk = 0;
  p->ekkk = 0;
  p->Adjust = 0;
  p->speedNow = 0;
  p->speedSet = 0;
}

// PID���㣬�����ⵥ��PID�Ŀ�����
// ��ڲ���������ֵ��ʵ��ֵ��PID��Ԫ�ṹ��
// �� �� ֵ��PID������
s32 PID_common(int set, int jiance, struct pid_uint *p)
{
  int ek = 0, U_k = 0;

  ek = jiance - set;

  U_k = p->U_kk + p->Kp * (ek - p->ekk) + p->Ki * ek + p->Kd * (ek - 2 * p->ekk + p->ekkk);

  p->U_kk = U_k;
  p->ekkk = p->ekk;
  p->ekk = ek;

  if (U_k > (p->Ur))
    U_k = p->Ur;
  if (U_k < -(p->Ur))
    U_k = -(p->Ur);

  return U_k >> 10;
}

void PID_Reset_Yaw(float yaw)
{
  pid_Yaw.SetPoint = yaw;
  s_offset = 0;
}

float PIDCal_car(float NextPoint)
{
  float dError, Error;
  double omega_rad;
  Error = pid_Yaw.SetPoint - NextPoint;           // ƫ��
  pid_Yaw.SumError += Error;                      // ����
  dError = pid_Yaw.LastError - pid_Yaw.PrevError; // ��ǰ΢��
  pid_Yaw.PrevError = pid_Yaw.LastError;
  pid_Yaw.LastError = Error;

  omega_rad = pid_Yaw.Proportion * Error          // ������
             + pid_Yaw.Integral * pid_Yaw.SumError// ������
             + pid_Yaw.Derivative * dError;       // ΢����

  if (omega_rad > PI / 6)
    omega_rad = PI / 6;
  if (omega_rad < -PI / 6)
    omega_rad = -PI / 6;

  return -omega_rad;
}

int PID_Get_Offset(void)
{
  return s_offset;
}

// pidѡ����
void Pid_Which(struct pid_uint *pl, struct pid_uint *pr, float yaw)
{
  s_offset = 0;

  // �����ٶ�pid
  if (pl->En == 1) {
    pl->Adjust = -PID_common(pl->speedSet, pl->speedNow, pl) - s_offset;
  } else {
    pl->Adjust = 0;
    reset_Uk(pl);
    pl->En = 2;
  }

  // �����ٶ�pid
  if (pr->En == 1) {
    pr->Adjust = -PID_common(pr->speedSet, pr->speedNow, pr) + s_offset;
  } else {
    pr->Adjust = 0;
    reset_Uk(pr);
    pr->En = 2;
  }
}

// �ٶȿ���PID����                                       
float Velocity_KP = 200, Velocity_KI = 150;

int Incremental_PI_B(int Encoder, int Target)
{
  static int Bias, Pwm, Last_bias;

  Bias = Target - Encoder;           // ����ƫ��
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; // ����ʽPI������
  if (Pwm > 3600) Pwm = 3600;
  if (Pwm < -3600) Pwm = -3600;
  Last_bias = Bias;                  // ������һ��ƫ��

  return Pwm;                        // �������
}

#define PID_AS_KP      150
#define PID_AS_KI      150

// ����ƽ���ٶ�PID
int Pid_Average_Speed_B(int target, int now)
{
  static int Bias = 0, Pwm = 0, Last_bias = 0, average = 0;
  static u32 count = 0;

  if (target == 0) {
    count = 0;
    Bias = 0;
    Pwm = 0;
    Last_bias = 0;
    average = 0;
    return 0;
  }

  count++;
  average = (average + now) / count;
  Bias = target - average;                //����ƫ��
  // ����ʽPI������
  Pwm += PID_AS_KP * (Bias - Last_bias) + PID_AS_KI * Bias;
  if (Pwm > 3600)  Pwm = 3600;
  if (Pwm < -3600) Pwm = -3600;
  Last_bias = Bias;                  //������һ��ƫ��

  return Pwm;                        //�������
}

// ����ƽ���ٶ�PID
int Pid_Average_Speed_D(int target, int now)
{
  static int Bias = 0, Pwm = 0, Last_bias = 0, average = 0;
  static u32 count = 0;

  if (target == 0) {
    count = 0;
    Bias = 0;
    Pwm = 0;
    Last_bias = 0;
    average = 0;
    return 0;
  }

  count++;
  average = (average + now) / count;
  Bias = target - average;           //����ƫ��
  // ����ʽPI������
  Pwm += PID_AS_KP * (Bias - Last_bias) + PID_AS_KI * Bias;
  if (Pwm > 3600)  Pwm = 3600;
  if (Pwm < -3600) Pwm = -3600;
  Last_bias = Bias;                  // ������һ��ƫ��

  return Pwm;                        // �������
}

// PID����
void Pid_Ctrl(int *leftMotor, int *rightMotor, float yaw)
{
  int temp_l = *leftMotor;
  int temp_r = *rightMotor;
  Pid_Which(&pid_Task_Left, &pid_Task_Right, yaw);
  temp_l += pid_Task_Left.Adjust;
  temp_r += pid_Task_Right.Adjust;

  if (temp_l > MOTOR_MAX_PULSE)  temp_l = MOTOR_MAX_PULSE;
  if (temp_l < -MOTOR_MAX_PULSE) temp_l = -MOTOR_MAX_PULSE;
  if (temp_r > MOTOR_MAX_PULSE)  temp_r = MOTOR_MAX_PULSE;
  if (temp_r < -MOTOR_MAX_PULSE) temp_r = -MOTOR_MAX_PULSE;

  *leftMotor  = temp_l;
  *rightMotor = temp_r;
}

// ��������PID����
void Left_Pid_Update_Value(float kp, float ki, float kd)
{
  if (kp > 10 || ki > 10 || kd > 10) 
    return;

  f_kp = kp;
  f_ki = ki;
  f_kd = kd;

  // ��1024ת��Ϊ�������㣬���⸡��������
  // �����ٶ�pid
  pid_Task_Left.Kp = 1024 * f_kp;
  pid_Task_Left.Ki = 1024 * f_ki;
  pid_Task_Left.Kd = 1024 * f_kd;
  reset_Uk(&pid_Task_Left);
}

// ��������PID����
void Right_Pid_Update_Value(float kp, float ki, float kd)
{
  if (kp > 10 || ki > 10 || kd > 10) 
    return;

  f_kp = kp;
  f_ki = ki;
  f_kd = kd;

  // ��1024ת��Ϊ�������㣬���⸡��������
  // �����ٶ�pid
  pid_Task_Right.Kp = 1024 * f_kp;
  pid_Task_Right.Ki = 1024 * f_ki;
  pid_Task_Right.Kd = 1024 * f_kd;
  reset_Uk(&pid_Task_Right);
}
