/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dev_motor_gm6020.h"
#include "alg_pid.h"
#include "UART.h"
#include "Servo.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern GM6020_Motor_t gm6020_motor1; // 电机实例
extern uint8_t UART_RxFlag;          // 串口接收标志
extern char UART_ProcBuf[];          // 串口数据缓存
extern volatile uint8_t Key_GetNum;  // 按键标志
extern volatile uint32_t g_can_tx_ok;    //【诊断用】CAN 发送成功次数
extern volatile uint32_t g_can_tx_fail;  //【诊断用】CAN 发送失败次数

PID_Typedef motor_pid;               // 速度环PID结构体
float target_rpm = 0.0f;             // 目标转速
/* USER CODE END Variables */
/* Definitions for Offline_Detecti */
osThreadId_t Offline_DetectiHandle;
const osThreadAttr_t Offline_Detecti_attributes = {
  .name = "Offline_Detecti",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Key_ServoTask */
osThreadId_t Key_ServoTaskHandle;
const osThreadAttr_t Key_ServoTask_attributes = {
  .name = "Key_ServoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for PrintMotorTask */
osThreadId_t PrintMotorTaskHandle;
const osThreadAttr_t PrintMotorTask_attributes = {
  .name = "PrintMotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void OfflineTask(void *argument);
void ServoTask(void *argument);
void PrintMotorSpeed(void *argument);
void Motor(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Offline_Detecti */
  Offline_DetectiHandle = osThreadNew(OfflineTask, NULL, &Offline_Detecti_attributes);

  /* creation of Key_ServoTask */
  Key_ServoTaskHandle = osThreadNew(ServoTask, NULL, &Key_ServoTask_attributes);

  /* creation of PrintMotorTask */
  PrintMotorTaskHandle = osThreadNew(PrintMotorSpeed, NULL, &PrintMotorTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(Motor, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_OfflineTask */
/**
  * @brief  Function implementing the Offline_Detecti thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_OfflineTask */
void OfflineTask(void *argument)
{
  /* USER CODE BEGIN OfflineTask */
  /* Infinite loop */
  int16_t red_pwm = 0;
  int8_t dir = 1;

  for (;;)
    {
      if (gm6020_motor1.offline_flag)
      {
        //离线：红灯呼吸，关闭绿灯
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

        // 呼吸计算：0->500 用 1 秒，来回共 2 秒（TIM5 的 ARR=499，500 即满占空比）
        red_pwm += dir;
        if (red_pwm >= 500)
        {
          red_pwm = 500;
          dir = -1;
        }
        else if (red_pwm <= 0)
        {
          red_pwm = 0;
          dir = 1;
        }
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, red_pwm);
      }
      else
      {
        //正常常亮绿灯，关闭红灯
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 500);
      }

      osDelay(2); // 2ms 一次，0->500 耗时 1 秒，来回正好 2 秒
    }
  /* USER CODE END OfflineTask */
}

/* USER CODE BEGIN Header_ServoTask */
/**
* @brief Function implementing the Key_ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoTask */
void ServoTask(void *argument)
{
  /* USER CODE BEGIN ServoTask */
  /* Infinite loop */
  for(;;)
  {
    if (Key_GetNum == 1)
    {
      Key_GetNum = 0; //清除标志位
      Servo_Step();   // 调用修改后每次加20度的函数
    }

    osDelay(10);
  }
  /* USER CODE END ServoTask */
}

/* USER CODE BEGIN Header_PrintMotorSpeed */
/**
* @brief Function implementing the PrintMotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PrintMotorSpeed */
void PrintMotorSpeed(void *argument)
{
  /* USER CODE BEGIN PrintMotorSpeed */
  /* Infinite loop */
    uint32_t last_print_time = HAL_GetTick();
    static char tx_buf[64];   //DMA 发送期间缓冲区必须持续有效，不能用栈上的局部变量

    for(;;)
    {
      //解析指令: "motor+100" 或 "motor-100"
      if (UART_RxFlag)
      {
        UART_RxFlag = 0;
        if (strncmp(UART_ProcBuf, "motor", 5) == 0)
        {
          target_rpm = (float)atof(&UART_ProcBuf[5]);
        }
      }
      //每两秒打印一次信息
      if (HAL_GetTick() - last_print_time >= 2000)
      {
        sprintf(tx_buf, "RPM:%d TGT:%.1f %s OUT:%d TX:%lu/%lu\r\n",
                gm6020_motor1.speed_rpm, target_rpm,
                gm6020_motor1.offline_flag ? "Offline" : "Online",
                (int)motor_pid.out,
                (unsigned long)g_can_tx_ok, (unsigned long)g_can_tx_fail);
        UART_SendString(tx_buf);
        last_print_time = HAL_GetTick();
      }

      osDelay(10); // 让出CPU控制权
    }
  /* USER CODE END PrintMotorSpeed */
}

/* USER CODE BEGIN Header_Motor */
  /**
  * @brief Function implementing the MotorTask thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_Motor */
void Motor(void *argument)
{
  /* USER CODE BEGIN Motor */
    /* Infinite loop */
    for(;;)
    {
      GM6020_WDG(&gm6020_motor1);

      if (!gm6020_motor1.offline_flag)
      {
        float output = PID_Calc(&motor_pid, (float)gm6020_motor1.speed_rpm, target_rpm);
        GM6020_SendCommand((int16_t)output, 0, 0, 0);
      }
      else
      {
        PID_Reset(&motor_pid);
        GM6020_SendCommand(0, 0, 0, 0);
      }
      osDelay(2); // 500Hz控制频率
    }
  /* USER CODE END Motor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN1)
    {
      HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
      if (rx_header.StdId == 0x205) // GM6020 ID=1 对应的反馈报文ID为0x205
      {
        GM6020_Init(&gm6020_motor1, rx_data);
      }
    }
  }
/* USER CODE END Application */

