#include "Servo.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

extern TIM_HandleTypeDef htim1;
static uint16_t current_ccr = SERVO_MIN_CCR;
static int16_t  step_dir    = SERVO_STEP_CCR;

//通过下面这个函数先开启所有PWM通道
void Servo_Init(void)//开启并初始化PWM通道
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);//先让舵机归位
}

void Servo_Step(void)
{
    current_ccr += step_dir;
    if (current_ccr >= SERVO_MAX_CCR)
    {
      current_ccr = SERVO_MAX_CCR;
        step_dir = -SERVO_STEP_CCR;
    }
    else if (current_ccr <= SERVO_MIN_CCR)
    {
        current_ccr = SERVO_MIN_CCR;
        step_dir = SERVO_STEP_CCR;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_ccr);
}