//
// Created by 16051 on 2026/11/6.
//
#include "alg_pid.h"

/**
 * @brief  PID参数初始化
 * @param  pid传入要初始化的PID结构体指针
 * @note   把我们在CubeMonitor里调好的参数写进去，同时清空历史遗留数据
 */
void PID_Init(PID_Typedef *pid,float max_out,float max_iout,float kp,float ki,float kd) {
    pid->Kp=kp;
    pid->Ki=ki;
    pid->Kd=kd;
    pid->max_out=max_out;
    pid->max_iout=max_iout;
    //初始化时清空状态，防止内存中的数据残留使电机乱转
    PID_Reset(pid);
}

/**
 * @brief  PID核心计算函数
 * @param  pid传入PID结构体指针
 * @param  fdb实际反馈值
 * @param  set目标期望值
 * @retval 返回计算后的控制量
 */
float PID_Calc(PID_Typedef *pid,float fdb,float set) {
    pid->set = set;
    pid->fdb = fdb;

    //迭代误差
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->set - pid->fdb;//相当于target-now

    pid->Pout = pid->Kp * pid->error[0];//P项
    pid->Iout += pid->Ki * pid->error[0];//I项，不断累加
    pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);//D项

    //防止积分过大，积分限幅
    if (pid->Iout > pid->max_iout) {
        pid->Iout = pid->max_iout;
    }else if (pid->Iout < -pid->max_iout) {
        pid->Iout = -pid->max_iout;
    }

    pid->out = pid->Pout +  pid->Iout + pid->Dout;

    //保护硬件，防止总输出过大
    if (pid->out > pid->max_out) {
        pid->out = pid->max_out;
    }else if (pid->out < -pid->max_out) {
        pid->out = -pid->max_out;
    }

    return pid->out;
}

/**
 * @brief  清空PID历史状态
 * @note   在电机断线、急停或切换模式时调用，避免危险
 */
void PID_Reset(PID_Typedef *pid) {
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
}