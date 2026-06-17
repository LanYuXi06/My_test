#ifndef PID_H
#define PID_H

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"
//#include "arm_math.h"

typedef enum
{
	Pos_Pid,		/*λ��ʽPID*/
	Inc_Pid			/*����ʽPID*/
}Pid_Mode;

typedef struct
{
	Pid_Mode Mode;
	float Kp , Ki , Kd;
	float Exp , Real;
	float Max_Out , Max_Iout;
	float Pout , Iout , Dout , Tout;
	float Err[3];
}Pid_t;

bool Pid_Handle_Init(Pid_t *pid,const float Kpid[5],const Pid_Mode Mode);
float PID_Calculate(Pid_t *pid,const float real ,const float exp);
float PID_Calculate_Angle(Pid_t *pid,const float real,const float exp);
float PID_Calculate_Diff(Pid_t *pid,const float real,const float exp);
bool PID_Clear(Pid_t *pid);
#endif