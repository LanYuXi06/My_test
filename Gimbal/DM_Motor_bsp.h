#ifndef DM_MOTOR_BSP_H
#define DM_MOTOR_BSP_H

#include "FDCANHandler.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#define PI 3.14159265358979323846f
typedef enum
{
	Disable = 0,
	Enable = 1,
	AngleCalibrationErr = 3,
	SensorOutputErr	= 4,
	EncoderCalibrationErr = 5,
	ExceedVoltage = 8,
	UnderVoltage = 9,
	OverCurrent = 10,
	MosOverTemp	= 11,
	MotorOverTemp = 12,
	CommLoss			= 13,
	OverLoad			= 14
}DM_Motor_Status_t;

typedef enum
{
	MIT = 1,
	PosVel,
	Vel,
	ForPst
}DM_Motor_Control_Mode;

typedef enum
{
	Recv_Normal = 0,
	Recv_ControlMode,
	Recv_Pmax,
	Recv_Vmax,
	Recv_Imax,
	Recv_CurPos,
	Recv_OutPos,
	Recv_V_Kp,
	Recv_V_Ki,
	Recv_P_Kp,
	Recv_P_Ki,
	Recv_Count
}DM_Motor_Recv_Type;

typedef struct
{
	uint8_t Motor_Id;
	DM_Motor_Status_t status;
	float Position;
	float Velocity;
	float Current;
	float T_mos;
	float T_Rotor;
	float Pmax;
	float Vmax;
	float Imax;
	float CurPos;
	float OutPos;
	float V_Kp;
	float V_Ki;
	float P_Kp;
	float P_Ki;
	DM_Motor_Control_Mode mode;
	DM_Motor_Recv_Type recv_type;
}DM_Motor_Handle_t;

bool DM_Motor_Init(DM_Motor_Handle_t *dm);
void DM_Motor_Enable(DM_Motor_Handle_t *dm);
void DM_Motor_Diable(DM_Motor_Handle_t *dm);
void DM_Motor_Write_Current_Position_As_ZeroPoint(DM_Motor_Handle_t *dm);
void DM_Motor_Clear_Error(DM_Motor_Handle_t *dm);

void DM_Motor_Mit_Control(DM_Motor_Handle_t *dm,float pos,float vel,float Kp,float Kd,float I);
void DM_Motor_PosVel_Control(DM_Motor_Handle_t *dm,float position,float velocity);
void DM_Motor_Velovity_Control(DM_Motor_Handle_t *dm,float velocity);
void DM_Motor_ForPst_Control(DM_Motor_Handle_t *dm,float pos,uint16_t vel,uint16_t current);
void DM_Motor_Read_Register(uint8_t RID);
void DM_Motor_Write_Register(uint8_t RID,uint8_t *data);
void DM_Motor_Store_Register(void);
void DM_Motor_Modif_Mode(DM_Motor_Handle_t *dm,DM_Motor_Control_Mode mode);
void DM_Motor_Read_Pmax(DM_Motor_Handle_t *dm);
void DM_Motor_Write_Pmax(DM_Motor_Handle_t *dm,float pmax);
void DM_Motor_Read_Vmax(DM_Motor_Handle_t *dm);
void DM_Motor_Write_Vmax(DM_Motor_Handle_t *dm,float vmax);
void DM_Motor_Read_Imax(DM_Motor_Handle_t *dm);
void DM_Motor_Write_Imax(DM_Motor_Handle_t *dm,float imax);
void DM_Motor_Read_Current_Pos(DM_Motor_Handle_t *dm);
void DM_Motor_Read_Output_Pos(DM_Motor_Handle_t *dm);
void DM_Motor_Read_P_Ki(DM_Motor_Handle_t *dm);
void DM_Motor_Read_P_Kp(DM_Motor_Handle_t *dm);
void DM_Motor_Read_V_Ki(DM_Motor_Handle_t *dm);
void DM_Motor_Read_V_Kp(DM_Motor_Handle_t *dm);
void DM_Motor_Write_V_Ki(DM_Motor_Handle_t *dm,float V_Kp);
void DM_Motor_Write_V_Kp(DM_Motor_Handle_t *dm,float V_Ki);
void DM_Motor_Write_P_Ki(DM_Motor_Handle_t *dm,float P_Kp);
void DM_Motor_Write_P_Kp(DM_Motor_Handle_t *dm,float P_Ki);

void DM_Motor_RecvData(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_Mode(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_Pmax(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_Vmax(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_Imax(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_CurPos(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_OutPos(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_P_Ki(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_P_Kp(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_V_Ki(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
void DM_Motor_Recv_V_Kp(DM_Motor_Handle_t *dm,uint8_t *rxbuf);
	
#endif