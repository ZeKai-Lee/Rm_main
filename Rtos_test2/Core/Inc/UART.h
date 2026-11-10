#ifndef __UART_H
#define __UART_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "dev_motor_gm6020.h"

#define RX_BUF_SIZE 128

extern uint8_t UART_RxFlag;//接收完成标志位
extern uint16_t UART_RxLen;//接收数据有效长度
extern char UART_ProcBuf[RX_BUF_SIZE];//数据处理缓存区

/**
 * @brief  启动ReceiveToIdle DMA接收
 */
void UART_Init(void);

/**
 * @brief  串口简单发送函数，使用DMA避免阻塞
 * @param  str要发送的字符串
 */
void UART_SendString(const char* str);

/**
 * @brief  电机串口通信更新（指令解析与状态打印）
 * @param  p_target_rpm目标转速的指针，用于接收解析后的新转速
 * @param  motor指向电机结构体的指针，用于读取当前转速和离线状态
 * @note   需在RTOS任务中循环调用
 */
void UART_MotorComm_Update(float *p_target_rpm,GM6020_Motor_t *motor);

#endif //MDA_USART_UART_H
