//
// Created by 16051 on 2026/11/5.
//
#include "dev_motor_gm6020.h"

extern CAN_HandleTypeDef hcan1;//引用CAN句柄
GM6020_Motor_t gm6020_motor1;

/**
 * @brief  解析 GM6020 的 CAN 反馈报文，刷新电机状态
 * @param  motor   指向要更新的电机结构体
 * @param  rx_data 收到的 8 字节原始数据
 * @note   按 GM6020 协议依次拆出机械角度、转速、实际电流、温度（均为大端）；
 *          同时更新时间戳并清除离线标志，相当于"喂狗"
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
 * @brief  通过 CAN1 向 GM6020 发送电压控制指令
 * @param  motor1_vol  1 号电机的电压指令
 * @param  motor2_vol  2 号电机的电压指令
 * @param  motor3_vol  3 号电机的电压指令
 * @param  motor4_vol  4 号电机的电压指令
 * @note   报文 ID 固定 0x1FF，一帧同时控制 1~4 号电机；
 *          每个指令按高字节在前拆成 2 字节；有效范围约 ±30000，对应 ±24V
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

    HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data,&tx_mailbox);
}

/**
 * @brief  电机软件看门狗，判断电机是否掉线
 * @param  motor  指向需要检测的电机结构体
 * @note   距离上次收到反馈超过 GM6020_offline_timeout（50ms）就置离线标志；
 *          因此调用周期必须明显小于 50ms，现在是在 500Hz 的 Motor 任务里调用
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