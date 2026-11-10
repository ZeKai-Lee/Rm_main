//
// Created by 16051 on 2026/11/5.
//

#ifndef TEST_MOTOR_GM6020_H
#define TEST_MOTOR_GM6020_H

#include "main.h"

//宏定义电机得超时时间
#define GM6020_offline_timeout 50

typedef struct {
    uint16_t ecd;//转子的机械角度
    int16_t speed_rpm;//电机的转速
    int16_t given_current;//当前扭矩下的电流
    uint8_t temperature;//电机温度

    uint32_t last_update_time;//上次收到CAN报文的时间（用来刷新看门狗）
    uint8_t offline_flag;//状态标志，0在线，1离线
}GM6020_Motor_t;

extern GM6020_Motor_t gm6020_motor1;

void GM6020_Init(GM6020_Motor_t *motor,uint8_t *rx_data);
void GM6020_SendCommand(int16_t motor1_vol, int16_t motor2_vol, int16_t motor3_vol,int16_t motor4_vol);
void GM6020_WDG(GM6020_Motor_t *motor);

#endif //TEST_MOTOR_GM6020_H
