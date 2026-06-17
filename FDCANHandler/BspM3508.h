#ifndef BSPM3508_H
#define BSPM3508_H

#include "FDCANBsp.h"
#include "stdbool.h"
#include "PID.h"

typedef enum
{
	SingleRing = 0,
	DoubleRing
}M3508_Control_Mode_t;

typedef struct
{
  int16_t  Pre_Angle;
  int16_t  Last_Angle;
  int16_t  Angle;
  int16_t  Velocity;
  int16_t  Current;
  int16_t  Temperature;
}M3508_t;

typedef struct
{
	M3508_t m3508;
	Pid_t pid; 	
}M3508_SinRing_t;

typedef struct
{
	M3508_t m3508;
	Pid_t inspid;
	Pid_t extpid;
}M3508_DouRing_t;

typedef struct
{
	float	 Angle;
	float	 Total_Angle;
	float    Last_Angle;
	int16_t  Velocity;
	int16_t  Current;
	int16_t  Temperature;
}M3508_YawAxis_t;

typedef struct
{
	M3508_YawAxis_t yaw;
	Pid_t inspid;
	Pid_t extpid;
	uint8_t i;
}SteerYawAxis_t;

typedef struct
{
	SteerYawAxis_t yawaxis;
	M3508_SinRing_t wheel;
}Steering_Handle_t;

bool M3508_SinRing_Init(M3508_SinRing_t *m3508_handle,M3508_Control_Mode_t ctlmode,float pid[5],Pid_Mode pidmode);
bool M3508_DouRing_Init(M3508_DouRing_t *m3508_handle,M3508_Control_Mode_t ctlmode,float ipid[5],float epid[5],Pid_Mode ipidmode,Pid_Mode epidmode);
bool M3508_YawAxis_Handle_Init(SteerYawAxis_t *yaw,float ipid[3],float epid[3],Pid_Mode ipidmode,Pid_Mode epidmode,uint8_t time);
bool Steering_Handle_Init(Steering_Handle_t *steer,float wpid[5],float yipid[5],float yepid[5],Pid_Mode wpidmode,Pid_Mode ipidmode,Pid_Mode epidmode,uint8_t time);
bool M3508_Data_Resolving(M3508_t *m3508,const uint8_t *CAN_RX_BUF);
bool Steering_Resolving(Steering_Handle_t *steering,const uint8_t *CAN_RX_BUF);

#endif
