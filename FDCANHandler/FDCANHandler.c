#include "FDCANHandler.h"

FDCAN_TxTypeDef_t ChassisTxHandle;
FDCAN_TxTypeDef_t LkmtechTxHandle;
FDCAN_TxTypeDef_t SuperCapTxHandle;
FDCAN_TxTypeDef_t SuperCapInitHandle;

FDCAN_TxTypeDef_t DM_MotorPosVelTxHandle;
FDCAN_TxTypeDef_t DM_MotorVelTxHandle;
FDCAN_TxTypeDef_t DM_MotorForPosTxHandle;
FDCAN_TxTypeDef_t DM_MotorTxHandle;
FDCAN_TxTypeDef_t DM_MotorParaTxHandle;

void FDCAN_Init(void)
{
	FDCAN_Filter_Init();
	FDCAN_TxTypeDef_Init(&ChassisTxHandle,&hfdcan1,FDCAN_STANDARD_ID,ChassisTxId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_16,FDCAN_BRS_OFF,FDCAN_FD_CAN,Buffer,1);
	FDCAN_TxTypeDef_Init(&SuperCapTxHandle,&hfdcan2,FDCAN_STANDARD_ID,SendSuperCapId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_4,FDCAN_BRS_OFF,FDCAN_FD_CAN,Fifo,0);
	FDCAN_TxTypeDef_Init(&SuperCapInitHandle,&hfdcan2,FDCAN_STANDARD_ID,SuperCapInitId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_1,FDCAN_BRS_OFF,FDCAN_FD_CAN,Fifo,0);
	
//	FDCAN_TxTypeDef_Init(&LkmtechTxHandle,&hfdcan3,FDCAN_STANDARD_ID,LkmtechId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN,Fifo,0);
	
	FDCAN_TxTypeDef_Init(&DM_MotorTxHandle,&hfdcan3,FDCAN_STANDARD_ID,DM_MotorId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN,Fifo,0);
	
	FDCAN_TxTypeDef_Init(&DM_MotorPosVelTxHandle,&hfdcan3,FDCAN_STANDARD_ID,DM_MotorPosVelId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN,Fifo,0);
	
	FDCAN_TxTypeDef_Init(&DM_MotorVelTxHandle,&hfdcan3,FDCAN_STANDARD_ID,DM_MotorVelId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_4,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN,Fifo,0);
	
	FDCAN_TxTypeDef_Init(&DM_MotorForPosTxHandle,&hfdcan3,FDCAN_STANDARD_ID,DM_MotorForPosId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN,Fifo,0);
	
	FDCAN_TxTypeDef_Init(&DM_MotorParaTxHandle,&hfdcan3,FDCAN_STANDARD_ID,DM_MotorParaId,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN,Fifo,0);
}

HAL_StatusTypeDef SendChassisInstruction(const uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&ChassisTxHandle,txbuf);
}

//HAL_StatusTypeDef SendLkmtechInstruction(uint8_t *txbuf)
//{
//	return FDCAN_Send_Msg(&LkmtechTxHandle,txbuf);
//}

HAL_StatusTypeDef SendSuperCapInstruction(uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&SuperCapTxHandle,txbuf);
}

HAL_StatusTypeDef SendSuperCapInit(uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&SuperCapInitHandle,txbuf);
}

//DM_Motor
HAL_StatusTypeDef SendDM_MotorInstruction(const uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&DM_MotorTxHandle,txbuf);
}

HAL_StatusTypeDef SendDM_MotorPosVelInstruction(const uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&DM_MotorPosVelTxHandle,txbuf);
}

HAL_StatusTypeDef SendDM_MotorVelTxInstruction(const uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&DM_MotorVelTxHandle,txbuf);
}

HAL_StatusTypeDef SendDM_MotorForPosInstruction(const uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&DM_MotorForPosTxHandle,txbuf);
}

HAL_StatusTypeDef SendDM_MotorParaInstruction(const uint8_t *txbuf)
{
	return FDCAN_Send_Msg(&DM_MotorParaTxHandle,txbuf);
}
