#include "BspM3508.h"
#include "stdarg.h"

bool M3508_SinRing_Init(M3508_SinRing_t *m3508_handle,M3508_Control_Mode_t ctlmode,float pid[5],Pid_Mode pidmode)
{
	if(m3508_handle == NULL || pid == NULL) return false;
	Pid_Handle_Init(&m3508_handle->pid,pid,pidmode);
	m3508_handle->m3508.Pre_Angle = 0;
	m3508_handle->m3508.Last_Angle = 0;
	m3508_handle->m3508.Angle = 0;
	m3508_handle->m3508.Velocity = 0;
	m3508_handle->m3508.Current = 0;
	m3508_handle->m3508.Temperature = 0;
	return true;
}

bool M3508_DouRing_Init(M3508_DouRing_t *m3508_handle,M3508_Control_Mode_t ctlmode,float ipid[5],float epid[5],Pid_Mode ipidmode,Pid_Mode epidmode)
{
	if(m3508_handle == NULL || ipid == NULL || epid == NULL) return false;
	Pid_Handle_Init(&m3508_handle->inspid,ipid,ipidmode);
	Pid_Handle_Init(&m3508_handle->extpid,epid,epidmode);
	m3508_handle->m3508.Pre_Angle = 0;
	m3508_handle->m3508.Last_Angle = 0;
	m3508_handle->m3508.Angle = 0;
	m3508_handle->m3508.Velocity = 0;
	m3508_handle->m3508.Current = 0;
	m3508_handle->m3508.Temperature = 0;
	return true;
}

bool M3508_YawAxis_Handle_Init(SteerYawAxis_t *yaw,float ipid[3],float epid[3],Pid_Mode ipidmode,Pid_Mode epidmode,uint8_t time)
{	
	if(yaw == NULL || ipid == NULL || epid == NULL) return false;
	Pid_Handle_Init(&yaw->inspid,ipid,ipidmode);
	Pid_Handle_Init(&yaw->extpid,epid,epidmode);
	yaw->yaw.Angle = 0.0f;
	yaw->yaw.Last_Angle = 0.0f;
	yaw->yaw.Total_Angle = 0.0f;
	yaw->yaw.Velocity = 0;
	yaw->yaw.Current = 0;
	yaw->yaw.Temperature = 0;
	yaw->i = time;
	return true;
}

bool Steering_Handle_Init(Steering_Handle_t *steer,float wpid[5],float yipid[5],float yepid[5],
						Pid_Mode wpidmode,Pid_Mode ipidmode,Pid_Mode epidmode,uint8_t time)			
{
	if(steer == NULL || wpid == NULL || yipid == NULL || yepid == NULL) return false;
	if(!M3508_SinRing_Init(&steer->wheel,SingleRing,wpid,wpidmode)) return false;
	if(!M3508_YawAxis_Handle_Init(&steer->yawaxis,yipid,yepid,ipidmode,epidmode,time)) return false;
	return true;
}


bool M3508_Data_Resolving(M3508_t *m3508,const uint8_t *CAN_RX_BUF)
{
	if(m3508 == NULL || CAN_RX_BUF == NULL) return false;
	 int16_t delta = 0;
     m3508->Pre_Angle   = (int16_t)(CAN_RX_BUF[0] << 8 | CAN_RX_BUF[1]);
     m3508->Velocity    = (int16_t)(CAN_RX_BUF[2] << 8 | CAN_RX_BUF[3]);
     m3508->Current     = (int16_t)(CAN_RX_BUF[4] << 8 | CAN_RX_BUF[5]);
     m3508->Temperature = (int8_t)  CAN_RX_BUF[6];
	 delta						  =  m3508->Pre_Angle - m3508->Last_Angle;
	 if(delta > 4096) delta -=8192;
	 else if(delta <	-4096) delta += 8192;
	 m3508->Angle += delta;
	 m3508->Last_Angle  =  m3508->Pre_Angle;
	 return true;
}

bool Steering_Resolving(Steering_Handle_t *steer,const uint8_t *CAN_RX_BUF)
{
	if(steer == NULL || CAN_RX_BUF == NULL) return false;
	steer->wheel.m3508.Angle				= (int16_t)(CAN_RX_BUF[0] << 8 | CAN_RX_BUF[1]);
	steer->wheel.m3508.Velocity    = (int16_t)(CAN_RX_BUF[2] << 8 | CAN_RX_BUF[3]);
	steer->wheel.m3508.Current     = (int16_t)(CAN_RX_BUF[4] << 8 | CAN_RX_BUF[5]);
	steer->wheel.m3508.Temperature = (int16_t)CAN_RX_BUF[6];
	
	
	steer->yawaxis.yaw.Angle		    = (float)((CAN_RX_BUF[8] << 8 | CAN_RX_BUF[9])/65535.0f * 360.0f);
	steer->yawaxis.yaw.Velocity    = (int16_t)(CAN_RX_BUF[10] << 8 | CAN_RX_BUF[11]);
	steer->yawaxis.yaw.Current     = (int16_t)(CAN_RX_BUF[12] << 8 | CAN_RX_BUF[13]);
	steer->yawaxis.yaw.Temperature = (int16_t)CAN_RX_BUF[14];

	float angle_diff = steer->yawaxis.yaw.Angle - steer->yawaxis.yaw.Last_Angle;
	
	if(steer->yawaxis.yaw.Angle > 180.0f)steer->yawaxis.yaw.Angle -= 360.0f;
	else if(steer->yawaxis.yaw.Angle < -180.0f)steer->yawaxis.yaw.Angle += 360.0f;
	
	steer->yawaxis.yaw.Total_Angle += angle_diff;
	steer->yawaxis.yaw.Last_Angle = steer->yawaxis.yaw.Angle;
	return true;
}