#include"main.h"

volatile uint8_t Key_GetNum = 0;

//HAL库外部中断回调函数
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
