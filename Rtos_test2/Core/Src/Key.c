#include"main.h"

volatile uint8_t Key_GetNum = 0;

/**
 * @brief  外部中断回调，处理按键按下事件
 * @param  GPIO_PIN  触发本次中断的引脚编号
 * @note   只处理 PA0；用两次触发的时间差做 20ms 软件消抖，
 *          确认有效后置 Key_GetNum = 1，由 ServoTask 轮询消费后清零
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
    static uint32_t last_trigger_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(GPIO_PIN == GPIO_PIN_0)
    {
        if((current_time - last_trigger_time)>20)
        {
                Key_GetNum = 1;
                last_trigger_time = current_time;
        }
    }
}
