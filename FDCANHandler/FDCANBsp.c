#include "FDCANBsp.h"

void FDCAN_TxTypeDef_Init(FDCAN_TxTypeDef_t *txtypedef,FDCAN_HandleTypeDef *hcan,uint32_t idtype,uint32_t id,
                                        uint32_t txframtype,uint32_t lenth,uint32_t brs_state,uint32_t fdformat,
                                        FDCANTx_Mode_TypeDef mode,uint32_t bufferidx)
{
    txtypedef->hfdcan = hcan;
    txtypedef->fdcan_txhandle.IdType = idtype;
    txtypedef->fdcan_txhandle.Identifier = id;
    txtypedef->fdcan_txhandle.TxFrameType = txframtype;
    txtypedef->fdcan_txhandle.DataLength = lenth;
    txtypedef->fdcan_txhandle.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txtypedef->fdcan_txhandle.BitRateSwitch = brs_state;
    txtypedef->fdcan_txhandle.FDFormat = fdformat;
    txtypedef->fdcan_txhandle.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txtypedef->fdcan_txhandle.MessageMarker = 0;
    txtypedef->mode = mode;
    txtypedef->bufferidx = bufferidx;
}

HAL_StatusTypeDef FDCAN_Send_Msg(const FDCAN_TxTypeDef_t *txtypedef, const uint8_t *pdata)
{
    if(txtypedef->mode == Fifo)
    {
        return HAL_FDCAN_AddMessageToTxFifoQ(txtypedef->hfdcan,&(txtypedef->fdcan_txhandle),pdata);
    }else
    {
        HAL_StatusTypeDef status;
        status =  HAL_FDCAN_AddMessageToTxBuffer(txtypedef->hfdcan,&(txtypedef->fdcan_txhandle),pdata,txtypedef->bufferidx);
        if (status == HAL_OK)
        {
            HAL_FDCAN_EnableTxBufferRequest(txtypedef->hfdcan,txtypedef->mode);
        }
        return status;
    }
}