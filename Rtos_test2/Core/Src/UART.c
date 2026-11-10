#include "UART.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "dev_motor_gm6020.h"

static uint32_t last_print_time = 0;
uint8_t UART_RxBuf[RX_BUF_SIZE];
char UART_ProcBuf[RX_BUF_SIZE];
uint8_t UART_RxFlag = 0;//标志位
uint16_t UART_RxLen = 0;

/**
 * @brief  启动串口 1 的 DMA + 空闲中断接收
 * @note   用 ReceiveToIdle 模式，一帧数据发完、总线空闲时才触发一次回调，
 *          不需要在中断里逐字节判断帧头帧尾
 */
void UART_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)UART_RxBuf, RX_BUF_SIZE);
}

/**
 * @brief  串口 1 非阻塞发送字符串
 * @param  str  要发送的字符串，必须是以 '\0' 结尾的
 * @note   走 DMA，函数调用后立即返回，不会阻塞任务；
 *          因此 str 指向的缓冲区在发送完成之前不能被改写
 */
void UART_SendString(const char* str)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)str, strlen(str));
}

/**
 * @brief  串口空闲中断回调，一帧接收完成时触发
 * @param  huart  触发本次回调的串口句柄
 * @param  Size   本次收到的字节数
 * @note   把收到的数据拷进 UART_ProcBuf 并补 '\0'，置 UART_RxFlag 通知任务处理；
 *          随后立刻重新开启 DMA 接收，保证下一帧不会丢
 */
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

/**
 * @brief  电机串口通信更新：解析上位机指令 + 周期性打印运行状态
 * @param  p_target_rpm  目标转速的指针，解析出的数值会写回这里
 * @param  motor         指向电机结构体的指针，用于读取转速和离线状态
 * @note   指令格式为 "motor+100" / "motor-100"，用 atof 解析（能识别正负号）；
 *          用 HAL_GetTick 计时，每 2 秒打印一行；需在 RTOS 任务里循环调用，
 *          调用周期决定了指令响应延迟
 */
void UART_MotorComm_Update(float *p_target_rpm,GM6020_Motor_t *motor)
{
    static char tx_buf[64];   //DMA发送期间缓冲区必须持续有效，不能用栈上的局部变量

    if (UART_RxFlag)
    {
        UART_RxFlag = 0;
        if (strncmp(UART_ProcBuf, "motor", 5) == 0)
        {
            *p_target_rpm = (float)atof(&UART_ProcBuf[5]);
        }
    }
    //每两秒打印一次信息
    if (HAL_GetTick() - last_print_time >= 2000)
    {
        sprintf(tx_buf, "RPM: %d, Target: %.1f, Status: %s\r\n",
                gm6020_motor1.speed_rpm, *p_target_rpm,
                gm6020_motor1.offline_flag ? "Offline" : "Online");
        UART_SendString(tx_buf);
        last_print_time = HAL_GetTick();
    }

}