#ifndef DOUBLEBUFFER_H
#define DOUBLEBUFFER_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <string.h>

#define DOUBLE_BUFFER_SIZE    2        // 双缓冲数量

typedef struct Double_Buffer_t Double_Buffer_t;

// 双缓冲区管理器
struct Double_Buffer_t{
		uint8_t buffer_size;
    uint8_t *buffer_addr;                      
    volatile uint8_t ready_buffer;                        // 已就绪的缓冲区
		volatile uint8_t active_buffer;
    volatile bool overflow;                               // 溢出标志
    volatile uint16_t received_size;                      // 接收到的数据大小
		uint8_t* (*get_ready_buffer)(Double_Buffer_t *db);
		uint8_t* (*get_active_buffer)(Double_Buffer_t *db);
};

// UART管理器
typedef struct {
    UART_HandleTypeDef *huart;        // UART句柄
    Double_Buffer_t double_buffer;    // 双缓冲区
    TaskHandle_t process_task;        // 处理任务句柄
		unsigned long last_update_time;
		uint16_t overtime;
} Uart_Manager_t;

// UART管理器
typedef struct {
    UART_HandleTypeDef *huart;        // UART句柄
    Double_Buffer_t double_buffer;    // 双缓冲区
}Uart_Manager_NoneTask_t;


// 函数声明
bool Double_Buffer_Init(Double_Buffer_t *db,uint8_t *buffer,uint8_t size);
bool Uart_Manager_Init(UART_HandleTypeDef *huart,TaskHandle_t task_handle,Uart_Manager_t *um,uint8_t *buffer,uint8_t buffer_size,
											uint16_t over_time);
void Uart_Start_Reception(Uart_Manager_t *um);
void Uart_RxEvent_Callback_Handle(Uart_Manager_t *um, uint16_t size);
bool Release_Buffer(Uart_Manager_t *um);

#endif