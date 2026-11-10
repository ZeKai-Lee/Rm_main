#ifndef MDA_USART_UART_H
#define MDA_USART_UART_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define RX_BUF_SIZE 128

extern uint8_t UART_RxFlag;//接收完成标志位
extern uint16_t UART_RxLen;//接收数据有效长度
extern char UART_ProcBuf[RX_BUF_SIZE];//数据处理缓存区

void UART_Init(void);
void UART_SendString(const char* str);

#endif //MDA_USART_UART_H
