#include "FDCANFilter.h"
#include "FDCANHandler.h"

void FDCAN_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
	
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterID1 = SteerLeftFrontId;
    sFilterConfig.FilterID2 = SteerLeftRearId;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

		HAL_FDCAN_ConfigFilter(&hfdcan1,&sFilterConfig);
	  HAL_FDCAN_Start(&hfdcan1);
		HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

		sFilterConfig.FilterIndex = 0;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterID1 = SuperCapId;
    sFilterConfig.FilterID2 = 0x00;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		
	  HAL_FDCAN_ConfigFilter(&hfdcan2,&sFilterConfig);
		
		sFilterConfig.FilterIndex = 1;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterID1 = VTMDataId;
    sFilterConfig.FilterID2 = VTMCrcErrId;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		
	  HAL_FDCAN_ConfigFilter(&hfdcan2,&sFilterConfig);
		
		sFilterConfig.FilterIndex = 2;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterID1 = HipnucId;
    sFilterConfig.FilterID2 = AutoLaunchId;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		
	  HAL_FDCAN_ConfigFilter(&hfdcan2,&sFilterConfig);
	  HAL_FDCAN_Start(&hfdcan2);
		HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	
		sFilterConfig.FilterIndex = 0;
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
//	  sFilterConfig.FilterID1 = LkmtechId; 
		sFilterConfig.FilterID1 = DM_MotorFBId;
	  sFilterConfig.FilterID2 = 0x00;
	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		
		HAL_FDCAN_ConfigFilter(&hfdcan3,&sFilterConfig);
	  HAL_FDCAN_Start(&hfdcan3);

		HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
		
		
}