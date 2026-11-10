#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#define SERVO_MIN_CCR   500   //0度对应的 CCR 最小角度
#define SERVO_MAX_CCR   2500  //180度对应的 CCR 最大角度
#define SERVO_STEP_CCR  222 //20度对应的步进值 步进角度

void Servo_Init(void);
void Servo_Step(void);

#endif
