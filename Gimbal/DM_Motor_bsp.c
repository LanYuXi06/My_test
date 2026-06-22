#include "DM_Motor_bsp.h"

#define Control_Mode_Register 0x0A
#define Pmax_Register					0x15
#define Vmax_Register					0x16
#define Imax_Register					0x17
#define Current_Pos_Register  0x50
#define Output_Pos_Register		0x51
#define V_Kp_Register					0x19
#define V_Ki_Register					0x1A
#define P_Kp_Register					0x1B
#define P_Ki_Register					0x1C

//#define Pmax 12.5f
//#define Vmax 30.0f
//#define Imax 10.0f

bool DM_Motor_Init(DM_Motor_Handle_t *dm)
{
	if(dm == NULL) return false;	
	dm->Motor_Id = 0;
	dm->status	 = 0;
	dm->Position = 0.0f;
	dm->Velocity = 0.0f;
	dm->Current = 0.0f;
	dm->T_mos = 0.0f;
	dm->T_Rotor = 0.0f;
	dm->Pmax = 0.0f;
	dm->Vmax = 0.0f;
	dm->Imax = 0.0f;
	dm->CurPos = 0.0f;
	dm->OutPos = 0.0f;
	dm->V_Kp = 0.0f;
	dm->V_Ki = 0.0f;
	dm->P_Kp = 0.0f;
	dm->P_Ki = 0.0f;
	dm->mode = 0;
	dm->recv_type = Recv_Normal;
	return true;
}

void DM_Motor_Enable(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_Normal;
	uint8_t txbuf[8];
	
	txbuf[0] = 0xFF; 
	txbuf[1] = 0xFF;
	txbuf[2] = 0xFF;
	txbuf[3] = 0xFF;
	txbuf[4] = 0xFF;
	txbuf[5] = 0xFF;
	txbuf[6] = 0xFF;
	txbuf[7] = 0xFC;
	
	SendDM_MotorInstruction(txbuf);
}

void DM_Motor_Diable(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_Normal;
	uint8_t txbuf[8];
	
	txbuf[0] = 0xFF; 
	txbuf[1] = 0xFF;
	txbuf[2] = 0xFF;
	txbuf[3] = 0xFF;
	txbuf[4] = 0xFF;
	txbuf[5] = 0xFF;
	txbuf[6] = 0xFF;
	txbuf[7] = 0xFD;
	
	SendDM_MotorInstruction(txbuf);
}

void DM_Motor_Write_Current_Position_As_ZeroPoint(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_Normal;
	uint8_t txbuf[8];
	
	txbuf[0] = 0xFF; 
	txbuf[1] = 0xFF;
	txbuf[2] = 0xFF;
	txbuf[3] = 0xFF;
	txbuf[4] = 0xFF;
	txbuf[5] = 0xFF;
	txbuf[6] = 0xFF;
	txbuf[7] = 0xFE;
	
	SendDM_MotorInstruction(txbuf);
}

void DM_Motor_Clear_Error(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_Normal;
	uint8_t txbuf[8];
	
	txbuf[0] = 0xFF; 
	txbuf[1] = 0xFF;
	txbuf[2] = 0xFF;
	txbuf[3] = 0xFF;
	txbuf[4] = 0xFF;
	txbuf[5] = 0xFF;
	txbuf[6] = 0xFF;
	txbuf[7] = 0xFB;
	
	SendDM_MotorInstruction(txbuf);
}

void DM_Motor_Mit_Control(DM_Motor_Handle_t *dm,float pos,float vel,float Kp,float Kd,float I)
{
	int16_t int_pos = pos / 12.5f * 32768 + 32767;
	int16_t int_vel = vel /	28.0f * 2048 + 2047;
	int16_t int_i	  = I / 12.0f * 2048 + 2047;
	int16_t int_kp  = Kp / 500.0f * 4095;
	int16_t int_kd  = Kd / 5.0f * 4095;
	
	uint8_t txbuf[8];
	txbuf[0] = (int_pos >> 8) & 0xFF;
	txbuf[1] = int_pos & 0xFF;
	txbuf[2] = (int_vel >> 4) & 0xFF;
	txbuf[3] = ((int_vel & 0x0F) << 4)|((int_kp >> 8) & 0x0F);
	txbuf[4] = int_kp & 0xFF;
	txbuf[5] = (int_kd >> 4) & 0xFF;
	txbuf[6] = ((int_kd & 0x0F) << 4)|((int_i >> 8) & 0x0F);
	txbuf[7] = int_i & 0xFF;
	
	SendDM_MotorInstruction(txbuf);
}

void DM_Motor_PosVel_Control(DM_Motor_Handle_t *dm,float position,float velocity)
{
	dm->recv_type = Recv_Normal;
	uint8_t txbuf[8];
	
	memcpy(&txbuf[0],&position,sizeof(float));
	memcpy(&txbuf[4],&velocity,sizeof(float));
	
	SendDM_MotorPosVelInstruction(txbuf);
}

void DM_Motor_Velovity_Control(DM_Motor_Handle_t *dm,float velocity)
{
	dm->recv_type = Recv_Normal;
	uint8_t txbuf[4];
	
	memcpy(&txbuf,&velocity,sizeof(float));
	SendDM_MotorVelTxInstruction(txbuf);
}

void DM_Motor_ForPst_Control(DM_Motor_Handle_t *dm,float pos,uint16_t vel,uint16_t current)
{
	dm->recv_type = Recv_Normal;
	uint8_t txbuf[8];
	memcpy(&txbuf[0],&pos,sizeof(float));
	memcpy(&txbuf[4],&vel,sizeof(uint16_t));
	memcpy(&txbuf[6],&current,sizeof(uint16_t));
	SendDM_MotorForPosInstruction(txbuf);
}

void DM_Motor_Read_Register(uint8_t RID)
{
	uint8_t txbuf[8] = {0};
	txbuf[0] = DM_MotorId & 0xFF;
	txbuf[1] = DM_MotorId>>8 & 0xFF;
	txbuf[2] = 0x33;
	txbuf[3] = RID;
	SendDM_MotorParaInstruction(txbuf);
}

void DM_Motor_Write_Register(uint8_t RID,uint8_t *data)
{
	uint8_t txbuf[8] = {0};
	
	txbuf[0] = DM_MotorId & 0xFF;
	txbuf[1] = DM_MotorId>>8 & 0xFF;
	txbuf[2] = 0x55;
	txbuf[3] = RID;
	txbuf[4] = data[0];
	txbuf[5] = data[1];
	txbuf[6] = data[2];
	txbuf[7] = data[3];
	
	SendDM_MotorParaInstruction(txbuf);
}

void DM_Motor_Store_Register(void)
{
	uint8_t txbuf[8] = {0};
	
	txbuf[0] = DM_MotorId & 0xFF;
	txbuf[1] = DM_MotorId>>8 & 0xFF;
	txbuf[2] = 0xAA;
	txbuf[3] = 0x01;

	SendDM_MotorParaInstruction(txbuf);
}

void DM_Motor_Read_Mode(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_ControlMode;
	uint8_t txbuf[4] = {0};
	DM_Motor_Read_Register(Control_Mode_Register);
}

void DM_Motor_Modif_Mode(DM_Motor_Handle_t *dm,DM_Motor_Control_Mode mode)
{
	dm->recv_type = Recv_ControlMode;
	uint8_t txbuf[4] = {0};
	txbuf[0] = mode;
	DM_Motor_Write_Register(Control_Mode_Register,txbuf);
}

void DM_Motor_Read_Pmax(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_Pmax;
	DM_Motor_Read_Register(Pmax_Register);
}

void DM_Motor_Write_Pmax(DM_Motor_Handle_t *dm,float pmax)
{
	dm->recv_type = Recv_Pmax;
	uint32_t p;
	memcpy(&p,&pmax,sizeof(float));
	
	DM_Motor_Write_Register(Pmax_Register,(uint8_t *)&p);
}

void DM_Motor_Read_Vmax(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_Vmax;
	DM_Motor_Read_Register(Vmax_Register);
}

void DM_Motor_Write_Vmax(DM_Motor_Handle_t *dm,float vmax)
{
	dm->recv_type = Recv_Vmax;
	uint32_t v;
	memcpy(&v,&vmax,sizeof(float));
	
	DM_Motor_Write_Register(Vmax_Register,(uint8_t *)&v);
}

void DM_Motor_Read_Imax(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_Imax;
	DM_Motor_Read_Register(Imax_Register);
}

void DM_Motor_Write_Imax(DM_Motor_Handle_t *dm,float imax)
{
	dm->recv_type = Recv_Imax;
	uint32_t i;
	memcpy(&i,&imax,sizeof(float));
	
	DM_Motor_Write_Register(Imax_Register,(uint8_t *)&i);
}

void DM_Motor_Read_Current_Pos(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_CurPos;
	DM_Motor_Read_Register(Output_Pos_Register);
}

void DM_Motor_Read_Output_Pos(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_OutPos;
	DM_Motor_Read_Register(Output_Pos_Register);
}

void DM_Motor_Read_V_Kp(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_V_Kp;
	DM_Motor_Read_Register(V_Kp_Register);
}

void DM_Motor_Read_V_Ki(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_V_Ki;
	DM_Motor_Read_Register(V_Ki_Register);
}

void DM_Motor_Read_P_Kp(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_P_Kp;
	DM_Motor_Read_Register(P_Kp_Register);
}

void DM_Motor_Read_P_Ki(DM_Motor_Handle_t *dm)
{
	dm->recv_type = Recv_P_Ki;
	DM_Motor_Read_Register(P_Ki_Register);
}

void DM_Motor_Write_V_Kp(DM_Motor_Handle_t *dm,float V_Kp)
{
	dm->recv_type = Recv_V_Kp;
	uint8_t txbuf[4];
	memcpy(txbuf,&V_Kp,sizeof(float));
	DM_Motor_Write_Register(V_Kp_Register,txbuf);
}

void DM_Motor_Write_V_Ki(DM_Motor_Handle_t *dm,float V_Ki)
{
	dm->recv_type = Recv_V_Ki;
	uint8_t txbuf[4];
	memcpy(txbuf,&V_Ki,sizeof(float));
	DM_Motor_Write_Register(V_Ki_Register,txbuf);
}

void DM_Motor_Write_P_Kp(DM_Motor_Handle_t *dm,float P_Kp)
{
	dm->recv_type = Recv_P_Kp;
	uint8_t txbuf[4];
	memcpy(txbuf,&P_Kp,sizeof(float));
	DM_Motor_Write_Register(P_Kp_Register,txbuf);
}

void DM_Motor_Write_P_Ki(DM_Motor_Handle_t *dm,float P_Ki)
{
	dm->recv_type = Recv_P_Ki;
	uint8_t txbuf[4];
	memcpy(txbuf,&P_Ki,sizeof(float));
	DM_Motor_Write_Register(P_Ki_Register,txbuf);
}


float int_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void DM_Motor_RecvData(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{  
	int16_t raw_pos = (int16_t)((rxbuf[1] << 8) | rxbuf[2]);
	int16_t raw_vel = (int16_t)(rxbuf[3] << 4) | ((rxbuf[4]>>4) & 0xF);
	int16_t raw_cur = (int16_t)((rxbuf[4] & 0xF)<<8 | rxbuf[5]);
	
	dm->Motor_Id =  rxbuf[0] & 0xF;
	dm->status      = (rxbuf[0] >> 4) & 0xF;
	
//	dm->Position = int_to_float(raw_pos,-dm->Pmax,dm->Pmax,16);
//	dm->Velocity = int_to_float(raw_vel,-dm->Vmax,dm->Vmax,12);
//	dm->Current  = int_to_float(raw_cur,-dm->Imax,dm->Imax,12);
//	dm->Position = raw_pos / 32767.0f * dm->Pmax;
//	dm->Position = dm->Position * 180.0f / PI;
//	dm->Velocity = (raw_vel / 2047.0f * dm->Vmax) - dm->Vmax;
//	dm->Current  = (raw_cur / 2047.0f * dm->Imax) - dm->Imax;
	dm->Position = raw_pos / 32767.0f * PI;
	dm->Position = dm->Position * 180.0f / PI;
	dm->Velocity = (raw_vel / 2047.0f * 28.0f) - 28.0f;
	dm->Current  = (raw_cur / 2047.0f * 12.0f) - 12.0f;
	
	dm->T_mos 	 = (float)rxbuf[6];
	dm->T_Rotor  = (float)rxbuf[7];
}

void DM_Motor_Recv_Mode(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	dm->mode = rxbuf[4];
}

void DM_Motor_Recv_Pmax(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->Pmax,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_Vmax(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->Vmax,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_Imax(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->Imax,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_CurPos(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->CurPos,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_OutPos(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->OutPos,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_V_Kp(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->V_Kp,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_V_Ki(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->V_Ki,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_P_Kp(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->P_Kp,&rxbuf[4],sizeof(float));
}

void DM_Motor_Recv_P_Ki(DM_Motor_Handle_t *dm,uint8_t *rxbuf)
{
	memcpy(&dm->P_Ki,&rxbuf[4],sizeof(float));
}


