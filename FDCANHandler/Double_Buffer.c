#include "Double_Buffer.h"

static inline uint8_t* Get_Ready_Buffer(Double_Buffer_t *db);
static inline uint8_t* Get_Active_Buffer(Double_Buffer_t *db);
  
// 初始化双缓冲区
bool Double_Buffer_Init(Double_Buffer_t *db,uint8_t *buffer,uint8_t buffer_size) {
    if (db == NULL || buffer == NULL) {
        return false;
		}
		db->buffer_size = buffer_size;
    db->buffer_addr = buffer;
    
    db->active_buffer = 0;      // DMA当前写入缓冲区0
    db->ready_buffer = 0xFF;    // 0xFF表示没有就绪缓冲区
    db->overflow = false;
    db->received_size = 0;
		db->get_active_buffer = Get_Active_Buffer;
		db->get_ready_buffer = Get_Ready_Buffer;
    return true;
}

// 初始化UART管理器
bool Uart_Manager_Init(UART_HandleTypeDef *huart,TaskHandle_t task_handle,Uart_Manager_t *um,uint8_t *buffer,uint8_t buffer_size,
											uint16_t over_time) 
	{
    if (um == NULL || huart == NULL) {
        return false;
    }
    
    um->huart = huart;
    um->process_task = task_handle;
		um->last_update_time = 0;
		um->overtime = over_time;
    
    // 初始化双缓冲区
    if (!Double_Buffer_Init(&(um->double_buffer),buffer,buffer_size)) {
        return false;
    }
		
		Uart_Start_Reception(um);
    
    return true;
}

// 启动DMA接收
void Uart_Start_Reception(Uart_Manager_t *um) {
    if (um == NULL || um->huart == NULL) {
        return;
    }
    
    // 停止之前的DMA（如果有）
    HAL_UART_DMAStop(um->huart);
    
    // 重置缓冲区状态
    um->double_buffer.active_buffer = 0;
    um->double_buffer.overflow = false;
    um->double_buffer.received_size = 0;
    
    // 启动DMA接收，使用第一个缓冲区
    // 注意：HAL_UARTEx_ReceiveToIdle_DMA使用正常模式，接收完成后会自动停止
    HAL_UARTEx_ReceiveToIdle_DMA(um->huart, 
                                um->double_buffer.get_active_buffer(&(um->double_buffer)), 
                                um->double_buffer.buffer_size);
		__HAL_DMA_DISABLE_IT(um->huart->hdmarx,DMA_IT_HT);	/*禁止DMA传输过半中断*/
}

// UART接收事件回调（在HAL_UARTEx_RxEventCallback中调用）
void Uart_RxEvent_Callback_Handle(Uart_Manager_t *um, uint16_t size) {
    if (um == NULL || size == 0 || size != um->double_buffer.buffer_size) {
        // 数据异常，重新启动接收
        Uart_Start_Reception(um);
        return;
    }
		um->last_update_time = xTaskGetTickCountFromISR();
		
    // 获取当前活动缓冲区
    uint8_t current_buffer = um->double_buffer.active_buffer;
    
    // 标记缓冲区就绪
    um->double_buffer.received_size = size;
    
    // 检查是否发生溢出（上一个缓冲区还未处理）
    if (um->double_buffer.ready_buffer != 0xFF) {
        um->double_buffer.overflow = true;
    }
    
    // 设置就绪缓冲区
    um->double_buffer.ready_buffer = current_buffer;
    
    // 切换到下一个缓冲区
    uint8_t next_buffer = (current_buffer + 1) % DOUBLE_BUFFER_SIZE;
    um->double_buffer.active_buffer = next_buffer;
		
		// 重新启动DMA接收，使用下一个缓冲区
    HAL_UARTEx_ReceiveToIdle_DMA(um->huart, 
                                um->double_buffer.get_active_buffer(&(um->double_buffer)), 
                                um->double_buffer.buffer_size);
		__HAL_DMA_DISABLE_IT(um->huart->hdmarx,DMA_IT_HT);	/*禁止DMA传输过半中断*/
    
    // 发送任务通知
    if (um->process_task != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(um->process_task, 
													0,      // 通知值
                          eNoAction, 
                          &xHigherPriorityTaskWoken);
        
        if (xHigherPriorityTaskWoken != pdFALSE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

static inline uint8_t* Get_Active_Buffer(Double_Buffer_t *db)
{
	if(db == NULL) return false;
	return db->buffer_addr + db->active_buffer * db->buffer_size;
}

static inline uint8_t* Get_Ready_Buffer(Double_Buffer_t *db)
{
	if(db == NULL || db->ready_buffer == 0xFF) return false;
	
	return db->buffer_addr + db->ready_buffer * db->buffer_size;
}

// 释放缓冲区（任务处理完成后调用）
bool Release_Buffer(Uart_Manager_t *um) {
    if (um == NULL) {
        return false;
    }
    // 如果没有其他就绪缓冲区，清除ready_buffer标志
        um->double_buffer.ready_buffer = 0xFF;
    // 清除溢出标志
    um->double_buffer.overflow = false;
		return true;
}