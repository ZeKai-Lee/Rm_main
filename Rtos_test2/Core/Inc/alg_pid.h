//
// Created by 16051 on 2026/11/6.
//

#ifndef _ALG_PID_H
#define _ALG_PID_H

//PID控制结构体，封装了所有计算需要的状态变量
typedef struct {
    //调参系数
    float Kp;
    float Ki;
    float Kd;

    //限幅保护
    float max_out;//总输出最大值限幅
    float max_iout;//积分项最大值限幅

    //运行状态变量
    float set;//期望值
    float fdb;//反馈值（feedback）

    //计算过程变量
    float out;//总输出量
    float Pout;//比例项输出
    float Iout;//积分项输出
    float Dout;//微分项输出

    float error[2];//error数组用来记录误差，error[0]是最新误差，error[1]代表上一次误差
}PID_Typedef;

void PID_Init(PID_Typedef *pid,float max_out,float max_iout,float kp,float ki,float kd);
float PID_Calc(PID_Typedef *pid,float fdb,float set);
void PID_Reset(PID_Typedef *pid);

#endif
