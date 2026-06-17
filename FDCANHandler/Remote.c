/**
  ****************************(C) COPYRIGHT 2025 征途****************************
  * @file       Remote.c/h
  * @brief      这里是英雄机器人的遥控器数据接收文件
  *             This is the remote control data receiving file for the hero robot
  * @note       
  * @history
  *  Version    Date               Author      			    Modification
  *  V1.1.0     May-13-2025      LinRuiYuan  	 			  	遥控器数据解析
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 征途****************************
  */
	
#include "Remote.h"

#define Remote_Max_Value     660
#define Remote_Middle_Value  0
#define Remote_Min_Value    -660

Remote_Channel RC;
uint8_t Remote_Buffer[Remote_Buffer_Size];

static uint32_t remote_last_update_time = 0;

void Remote_Reset(Remote_Channel *rc)
{
    rc->ch0 = Remote_Middle_Value;
    rc->ch1 = Remote_Middle_Value;
    rc->ch2 = Remote_Middle_Value;
    rc->ch3 = Remote_Middle_Value;
    rc->ch4 = Remote_Middle_Value;
    rc->s1 = 0;
    rc->s2 = 0;
}

void Remote_Init(Remote_Channel *rc)
{
    Remote_Reset(rc);

    remote_last_update_time = HAL_GetTick();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,
                                 Remote_Buffer,
                                 Remote_Buffer_Size);

    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

bool Remote_Data_Resolving(Remote_Channel *rc, const uint8_t *buf)
{
    if (rc == NULL || buf == NULL) return false;

    rc->ch0 = (((int16_t)(buf[0] | buf[1] << 8)) & 0x07FF) - 1024;
    rc->ch1 = (((int16_t)(buf[1] >> 3 | buf[2] << 5)) & 0x07FF) - 1024;
    rc->ch2 = (((int16_t)(buf[2] >> 6 | buf[3] << 2 | buf[4] << 10)) & 0x07FF) - 1024;
    rc->ch3 = (((int16_t)(buf[4] >> 1 | buf[5] << 7)) & 0x07FF) - 1024;
    rc->s1  = ((buf[5] >> 4) & 0x000C) >> 2;
    rc->s2  = (buf[5] >> 4) & 0x0003;
    rc->ch4 = (((int16_t)(buf[16] | buf[17] << 8)) & 0x07FF) - 1024;

    rc->ch4 = -rc->ch4;

    remote_last_update_time = HAL_GetTick();

    return true;
}

void Remote_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        if (Size == Remote_Buffer_Size)
        {
            Remote_Data_Resolving(&RC, Remote_Buffer);
        }
        else
        {
            Remote_Reset(&RC);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5,
                                     Remote_Buffer,
                                     Remote_Buffer_Size);

        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}

bool Remote_Is_Online(void)
{
    if (HAL_GetTick() - remote_last_update_time > Remote_Over_Time)
    {
        Remote_Reset(&RC);
        return false;
    }

    return true;
}
