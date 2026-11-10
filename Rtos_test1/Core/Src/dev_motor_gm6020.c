//
// Created by 16051 on 2026/11/5.
//
#include "dev_motor_gm6020.h"

extern CAN_HandleTypeDef hcan1;//引用CAN句柄
GM6020_Motor_t gm6020_motor1;

volatile uint32_t g_can_tx_ok   = 0;   //【诊断用】CAN 报文发送成功次数
volatile uint32_t g_can_tx_fail = 0;   //【诊断用】CAN 报文发送失败次数

/**
 *  @brief  解析GM6020反馈报文
 *  @param  motor 指向需要更新的电机结构体
 *  @param  rx_data CAN接收到的8字节数据数组
 *  @note   此函数应在CAN接收中断回调中调用
 */
void GM6020_Init(GM6020_Motor_t *motor,uint8_t *rx_data) {
    motor->ecd = (rx_data[0] << 8) | rx_data[1];
    motor->speed_rpm = (rx_data[2] << 8) | rx_data[3];
    motor->given_current = (rx_data[4] << 8) | rx_data[5];
    motor->temperature = rx_data[6];

    //更新时间戳喂狗
    motor->last_update_time = HAL_GetTick();
    motor->offline_flag = 0;
}

/**
 *  @brief  通过CAN发送控制指令给GM6020(ID:0x1FF控制电机1-4)
 *  @param  motor1_vol - motor4_vol:电压指令值
 */
void GM6020_SendCommand(int16_t motor1_vol,int16_t motor2_vol,int16_t motor3_vol,int16_t motor4_vol) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    tx_header.StdId = 0x1FF;//GM6020的1-4号电机控制ID
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    //数据按高低字节拆分装填
    tx_data[0] = motor1_vol >> 8;
    tx_data[1] = motor1_vol;
    tx_data[2] = motor2_vol >> 8;
    tx_data[3] = motor2_vol;
    tx_data[4] = motor3_vol >> 8;
    tx_data[5] = motor3_vol;
    tx_data[6] = motor4_vol >> 8;
    tx_data[7] = motor4_vol;

    if (HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data,&tx_mailbox) == HAL_OK)
    {
        g_can_tx_ok++;
    }
    else
    {
        g_can_tx_fail++;
    }
}

/**
 *  @brief  电机软件看门狗检测
 *  @param  motor指向需要检测的电机结构体
 *  @note   需在主循环或1ms定时器中高频调用
 */
void GM6020_WDG(GM6020_Motor_t *motor) {
    uint32_t current_time = HAL_GetTick();
    //如果当前时间减去上次更新时间超过了设定喂狗的时间，则判定为离线
    if ((current_time - motor->last_update_time) > GM6020_offline_timeout) {
        motor->offline_flag = 1;
    }
    else
    {
        motor->offline_flag = 0;
    }
}