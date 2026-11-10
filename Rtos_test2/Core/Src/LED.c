#include "LED.h"
#include "tim.h"
#include "main.h"

static int16_t red_pwm = 0;
static int8_t dir = 1;

/**
 * @brief  启动红绿两个 LED 的 PWM 通道并全部熄灭
 * @note   红灯用 TIM5_CH3（PH12），绿灯用 TIM5_CH2（PH11）；
 *          TIM5 的 ARR=499，所以 CCR 的有效范围是 0~500，500 即满占空比
 */
void LED_Init(void)
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);//Red
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);//Green

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
}

/**
 * @brief  根据电机在线状态刷新 LED 显示
 * @param  is_offline  电机状态，1 表示离线，0 表示在线
 * @note   离线时红灯做呼吸渐亮渐暗、绿灯熄灭；
 *          在线时绿灯常亮、红灯熄灭，并把呼吸变量复位，下次离线重新从头呼吸
 */
void LED_Update(uint8_t is_offline)
{
    if (is_offline)
    {
        //离线时红灯呼吸，绿灯关闭
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
        red_pwm += dir;
        if (red_pwm >= 1000)
        {
            red_pwm = 1000;
            dir = -1;
        }
        else if (red_pwm <= 0)
        {
            red_pwm = 0;
            dir = 1;
        }
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, red_pwm);
    }
    else//正常状态下绿灯常量，关闭红灯
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1000);
        //重置标志位
        red_pwm = 0;
        dir = 1;
    }
}