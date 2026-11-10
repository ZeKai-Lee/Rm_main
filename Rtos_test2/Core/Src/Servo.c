#include "Servo.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

extern TIM_HandleTypeDef htim1;
static uint16_t current_ccr = SERVO_MIN_CCR;
static int16_t  step_dir    = SERVO_STEP_CCR;

/**
 * @brief  启动舵机 PWM 通道并让舵机归位到 0 度
 * @note   使用 TIM1_CH1（PE9）；CCR=500 对应 0.5ms 脉宽，即最小角度位置
 */
void Servo_Init(void)//开启并初始化PWM通道
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);//先让舵机归位
}

/**
 * @brief  舵机步进一次，移动 20 度
 * @note   每次固定移动 SERVO_STEP_CCR（222 个计数，约 20 度）；
 *          撞到 0 度或 180 度边界时把步进方向取反，形成往返摆动
 */
void Servo_Step(void)
{
    //每次按键固定移动 20 度（222 个 CCR 计数），撞到 0 度或 180 度边界时自动换向
    int32_t next = (int32_t)current_ccr + step_dir;

    if (next >= SERVO_MAX_CCR)
    {
        next = SERVO_MAX_CCR;
        step_dir = -SERVO_STEP_CCR;
    }
    else if (next <= SERVO_MIN_CCR)
    {
        next = SERVO_MIN_CCR;
        step_dir = SERVO_STEP_CCR;
    }

    current_ccr = (uint16_t)next;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_ccr);
}