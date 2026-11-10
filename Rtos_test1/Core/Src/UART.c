#include "UART.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "usart.h"

uint8_t UART_RxBuf[RX_BUF_SIZE];
char UART_ProcBuf[RX_BUF_SIZE];
uint8_t UART_RxFlag = 0;//标志位
uint16_t UART_RxLen = 0;

//启动ReceiveToIdle DMA接收
void UART_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)UART_RxBuf, RX_BUF_SIZE);
}

//串口简单发送函数，避免在中断里阻塞
void UART_SendString(const char* str)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)str, strlen(str));
}

//空闲中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // 增加越界保护
        if (Size >= RX_BUF_SIZE)
        {
            Size = RX_BUF_SIZE - 1;
        }

        memcpy(UART_ProcBuf, UART_RxBuf, Size);
        UART_ProcBuf[Size] = '\0';
        UART_RxLen = Size;
        UART_ProcBuf[UART_RxLen] = '\0';
        UART_RxFlag = 1;

        // 再次开启 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)UART_RxBuf, RX_BUF_SIZE);
    }
}