//created by Caspian Sambei in 2026/3/1
#ifndef __PID_H
#define __PID_H


//全局变量应包含这个结构体类型
#define gimbal_mode     1
#define chassis_mode    1

// typedef struct{
//     float Target,Actual,Out;
//     float kp,ki,kd;
//     float LastError,PreError,SunError;
//     float I_L,I_M;
//     float OUTMAX,IMAX,DEADZONE;
//     float D_OUT,LAST_D_OUT,LAST_OUT;
//     //一阶低通滤波系数
//     float RC_DF
//     // 前馈增益
//     float k1,k2,last_Target;
// }pid;

void PID_Update(pid *p,uint8_t mode);
void PID_Clear(pid *p);

#endif
