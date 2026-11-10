//
// Created by 16051 on 2026/11/6.
//
#include "alg_pid.h"

/**
 * @brief  速度环 PID 参数初始化
 * @param  pid       指向 PID 结构体的指针
 * @param  max_out   输出限幅值（GM6020 电压指令满量程约 ±30000）
 * @param  max_iout  积分项限幅值，防止积分饱和
 * @param  kp        比例系数
 * @param  ki        积分系数
 * @param  kd        微分系数
 * @note   把调好的参数写进结构体，随后清空历史状态，防止残留数据让电机乱转
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
 * @brief  PID 核心计算，每次调用算出一拍控制量
 * @param  pid  指向 PID 结构体的指针
 * @param  fdb  实际反馈值（当前转速，单位 rpm）
 * @param  set  目标期望值（目标转速，单位 rpm）
 * @retval 限幅后的控制量，可直接作为电压指令下发给电调
 * @note   误差 = 目标 - 反馈；P、I、D 三项相加后先做积分限幅、再做总输出限幅
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
 * @brief  清空 PID 的全部历史状态
 * @param  pid  指向 PID 结构体的指针
 * @note   电机离线、急停或切换控制模式时调用，避免积分和上拍误差残留导致下一次上电乱冲
 */
void PID_Reset(PID_Typedef *pid) {
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
}