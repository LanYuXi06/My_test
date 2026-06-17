#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#define Remote_Buffer_Size 18
#define Remote_Over_Time   50

typedef struct
{
    int16_t ch0;   /*”““°∏ÀX÷·*/
    int16_t ch1;   /*”““°∏ÀY÷·*/
    int16_t ch2;   /*◊Û“°∏ÀX÷·*/
    int16_t ch3;   /*◊Û“°∏ÀY÷·*/
    int16_t ch4;   /*πˆ¬÷*/
    uint8_t s1;    /*◊Û≤‡≤¶∏À*/
    uint8_t s2;    /*”“≤‡≤¶∏À*/
} Remote_Channel;

extern Remote_Channel RC;
extern uint8_t Remote_Buffer[Remote_Buffer_Size];

void Remote_Init(Remote_Channel *rc);
void Remote_Reset(Remote_Channel *rc);
bool Remote_Data_Resolving(Remote_Channel *rc, const uint8_t *buf);
void Remote_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
bool Remote_Is_Online(void);

#endif
