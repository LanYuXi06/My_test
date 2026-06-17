#ifndef FDCANBSP_H
#define FDCANBSP_H

#include "stdlib.h"
#include "FDCANFilter.h"

typedef enum
{
    Fifo = 0,
    Buffer
}FDCANTx_Mode_TypeDef;

typedef struct 
{
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_TxHeaderTypeDef fdcan_txhandle;
    FDCANTx_Mode_TypeDef mode;
    uint32_t bufferidx;
}FDCAN_TxTypeDef_t;

void FDCAN_TxTypeDef_Init(FDCAN_TxTypeDef_t *txtypedef,FDCAN_HandleTypeDef *hcan,uint32_t idtype,uint32_t id,
                                        uint32_t txframtype,uint32_t lenth,uint32_t brs_state,uint32_t fdformat,
                                        FDCANTx_Mode_TypeDef mode,uint32_t bufferidx);

HAL_StatusTypeDef  FDCAN_Send_Msg(const FDCAN_TxTypeDef_t *txtypedef, const uint8_t *pdata);

#endif