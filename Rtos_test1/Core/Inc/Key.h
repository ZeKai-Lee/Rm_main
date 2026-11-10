#ifndef __KEY_H
#define __KEY_H

#include "main.h"
extern volatile uint8_t Key_GetNum;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN);

#endif
