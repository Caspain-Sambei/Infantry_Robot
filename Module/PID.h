//created by Caspian Sambei in 2026/3/1
#ifndef __PID_H
#define __PID_H


// typedef struct{
//     float Target,Actual,Out;
//     float kp,ki,kd;
//     float LastError,PreError,SunError;
//     float I_L,I_M;
//     float OUTMAX,IMAX,DEADZONE;
//     float D_OUT,LAST_D_OUT,LAST_OUT;
//     // 一阶低通滤波系数
//     float RC_DF;
//     // 前馈增益，k1为0是完全静态前馈，k1为1时完全动态前馈
//     float k1,last_Target;
// }PID_Structure;
#define I_FROZEN    1
#define I_CLEAR     2
void PID_Update(pid *p,uint8_t mode);
void PID_Init(pid *p,
              float kp,float ki,float kd,
              float OUTMAX,float IMAX,float DEADZONE,
              float I_L,float I_M,
              float RC_DF,float k1);
void PID_Clear(pid *p);

#endif
