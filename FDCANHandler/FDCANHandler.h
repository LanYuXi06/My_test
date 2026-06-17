#ifndef FDCANHANDLER_H
#define FDCANHANDLER_H

#include "FDCANBsp.h"

#define SuperCapId				0x2FF
#define SendSuperCapId		0x300
#define SuperCapInitId		0x301

#define VTMDataId					0x400
#define VTMOverTimeId			0x401
#define VTMCrcErrId				0x402
#define HipnucId						0x410
#define BelowBoardId				0x3FF

#define DM_MotorId					0x005
#define DM_MotorPosVelId		0x100 + DM_MotorId
#define DM_MotorVelId				0x200 + DM_MotorId
#define DM_MotorForPosId		0x300 + DM_MotorId
#define DM_MotorParaId			0x7FF
#define DM_MotorFBId				0x100

#define AutoLaunchId				0x520

#define ChassisTxId			 		0x6FF
#define SteerLeftFrontId  	0x701//左前
#define SteerRightFrontId 	0x702//右前
#define SteerRightRearId  	0x703//右后
#define SteerLeftRearId   	0x704//左后


void FDCAN_Init(void);
HAL_StatusTypeDef SendChassisInstruction(const uint8_t *txbuf);
//HAL_StatusTypeDef SendLkmtechInstruction(uint8_t *txbuf);
HAL_StatusTypeDef SendSuperCapInstruction(uint8_t *txbuf);
HAL_StatusTypeDef SendSuperCapInit(uint8_t *txbuf);

HAL_StatusTypeDef SendDM_MotorInstruction(const uint8_t *txbuf);
HAL_StatusTypeDef SendDM_MotorPosVelInstruction(const uint8_t *txbuf);
HAL_StatusTypeDef SendDM_MotorVelTxInstruction(const uint8_t *txbuf);
HAL_StatusTypeDef SendDM_MotorForPosInstruction(const uint8_t *txbuf);
HAL_StatusTypeDef SendDM_MotorParaInstruction(const uint8_t *txbuf);

#endif