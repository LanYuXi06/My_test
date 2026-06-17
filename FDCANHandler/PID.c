#include "PID.h"

#define Allowable_Error_Range 0.10f

static inline float Normalization_func(float exp, float real) {
    float error = exp - real;
    if (error > 180.0f){
        error -= 360.0f;
    } else if (error < -180.0f){
        error += 360.0f;
    }
    return error;
}

static inline float Limit_Max(float value , float max)
{
	if(value > max) value = max;
	if(value < -max) value = -max;
	return value;
}

bool Pid_Handle_Init(Pid_t *pid,const float Kpid[5],const Pid_Mode Mode)
{
	if(pid == NULL || Kpid == NULL) return false;
	pid->Mode = Mode;
	pid->Kp = Kpid[0];
	pid->Ki = Kpid[1];
	pid->Kd = Kpid[2];
	pid->Max_Iout = Kpid[3];
	pid->Max_Out = Kpid[4];
	pid->Err[0] = 0.0f;
	pid->Err[1] = 0.0f;
	pid->Err[2] = 0.0f;
	pid->Pout = 0.0f;
	pid->Iout = 0.0f;
	pid->Dout = 0.0f;
	pid->Tout = 0.0f;
	pid->Exp 	=	0.0f;
	pid->Real = 0.0f;
	return true;
}

float PID_Calculate(Pid_t *pid,const float real,const float exp)
{
	if(pid == NULL) return 0.0f;
	
	pid->Err[2] = pid->Err[1];
	pid->Err[1] = pid->Err[0];
	
	pid->Exp = exp;
	pid->Real = real;

	pid->Err[0] = exp - real;
	
	if(pid->Mode == Pos_Pid)
	{
		pid->Pout =  pid->Kp * pid->Err[0];
		pid->Iout += (pid->Ki * pid->Err[0]);
		pid->Dout =  pid->Kd * (pid->Err[0] - pid->Err[1]);
		
		pid->Iout = Limit_Max(pid->Iout,pid->Max_Iout);

		pid->Tout = pid->Pout + pid->Iout + pid->Dout;
		
		pid->Tout = Limit_Max(pid->Tout,pid->Max_Out);
	}else if(pid->Mode == Inc_Pid)
	{
		pid->Pout = pid->Kp * (pid->Err[0] - pid->Err[1]);
		pid->Iout = pid->Ki * pid->Err[0];
		
		pid->Iout = Limit_Max(pid->Iout,pid->Max_Iout);
		
		pid->Dout = pid->Kd * (pid->Err[0] - 2.0f*pid->Err[1] + pid->Err[2]);
		
		pid->Tout += pid->Pout + pid->Iout + pid->Dout;
		
		pid->Tout = Limit_Max(pid->Tout,pid->Max_Out);
	}
	return pid->Tout;
}


float PID_Calculate_Angle(Pid_t *pid,const float real,const float exp)
{
	if(pid == NULL) return 0.0f;
	
	pid->Err[2] = pid->Err[1];
	pid->Err[1] = pid->Err[0];
	
	pid->Exp = exp;
	pid->Real = real;

	pid->Err[0] = Normalization_func( exp,real);

	if(pid->Mode == Pos_Pid)
	{
		pid->Pout =  pid->Kp * pid->Err[0];
		pid->Iout += (pid->Ki * pid->Err[0]);
		pid->Dout = pid->Kd * (pid->Err[0] - pid->Err[1]);
		
		pid->Iout = Limit_Max(pid->Iout,pid->Max_Iout);
		
		pid->Tout = pid->Pout + pid->Iout + pid->Dout;

		pid->Tout = Limit_Max(pid->Tout,pid->Max_Out);
	}else if(pid->Mode == Inc_Pid)
	{
		pid->Pout = pid->Kp * (pid->Err[0] - pid->Err[1]);
		pid->Iout = pid->Ki * pid->Err[0];
		pid->Dout = (pid->Kd * (pid->Err[0] - 2.0f*pid->Err[1] + pid->Err[2]));
		
		pid->Tout += pid->Pout + pid->Iout + pid->Dout;
		
		pid->Tout = Limit_Max(pid->Tout,pid->Max_Out);
	}
	
//	float abs_err;
//	arm_abs_f32(&(pid->Err[0]),&abs_err,1);
	
//	if(abs_err < Allowable_Error_Range)pid->Tout = 0;
	return pid->Tout;
}

float PID_Calculate_Diff(Pid_t *pid,const float real,const float exp)
{
	if(pid == NULL) return 0.0f;
	
	static float last_real[3] = {0.0f};
	last_real[2] = last_real[1];
	last_real[1] = last_real[0];
	last_real[0] = 0.90f * real + 0.10f * last_real[1];
	
	pid->Err[2] = pid->Err[1];
	pid->Err[1] = pid->Err[0];

	pid->Exp = exp;
	pid->Real = last_real[0];

	pid->Err[0] = exp - last_real[0];
	
	if(pid->Mode == Pos_Pid)
	{
		pid->Pout =  pid->Kp * pid->Err[0];
		pid->Iout += (pid->Ki * pid->Err[0]);
		pid->Dout =  pid->Kd * (pid->Err[0] - pid->Err[1]);
		
		pid->Iout = Limit_Max(pid->Iout,pid->Max_Iout);

		pid->Tout = pid->Pout + pid->Iout + pid->Dout;
		
		pid->Tout = Limit_Max(pid->Tout,pid->Max_Out);
	}else if(pid->Mode == Inc_Pid)
	{
		pid->Pout = pid->Kp * (pid->Err[0] - pid->Err[1]);
		pid->Iout = pid->Ki * pid->Err[0];
		
		pid->Iout = Limit_Max(pid->Iout,pid->Max_Iout);
		
		pid->Dout = (pid->Kd * (last_real[0] - 2.0f*last_real[1] + last_real[2]));
		
		pid->Tout += pid->Pout + pid->Iout + pid->Dout;
		
		pid->Tout = Limit_Max(pid->Tout,pid->Max_Out);
	}
	return pid->Tout;
}

bool PID_Clear(Pid_t *pid)
{
	if(!pid)return false;
	pid->Err[0] = 0.0f;
	pid->Err[1] = 0.0f;
	pid->Err[2] = 0.0f;
	pid->Pout = 0.0f;
	pid->Iout = 0.0f;
	pid->Dout = 0.0f;
	pid->Tout = 0.0f;
	pid->Exp  = 0.0f;
	pid->Real = 0.0f;
	return true;
}




