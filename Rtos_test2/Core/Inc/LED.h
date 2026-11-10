#ifndef __LED_H
#define __LED_H

#include <stdint.h>

//初始化LED指示灯
void LED_Init(void);

//LED指示灯更新
void LED_Update(uint8_t is_offline);

#endif
