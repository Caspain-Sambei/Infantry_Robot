//
// Created by 18796 on 2026/2/23.
//

#ifndef GIMBAL_REG_H
#define GIMBAL_REG_H
#include <stdint.h>
#include "Filter.h"
/***************************************************************
 *                     遥控器通信
 ***************************************************************/
#include "bsp_rc.h"

/***************************************************************
 *                     视觉通信
 ***************************************************************/
typedef struct{
    float roll;
    float pitch;
    float yaw;
}SENDPACKET;

/***************************************************************
 *                     PID结构体
 ***************************************************************/
typedef struct{
    float Target,Actual,Out;
    float kp,ki,kd;
    float LastError,PreError,SunError;
    float I_L,I_M;
    float OUTMAX,IMAX,DEADZONE;
    float D_OUT,LAST_D_OUT,LAST_OUT;
    // 一阶低通滤波系数
    float RC_DF;
    // 前馈增益
    float k1,last_Target;
}pid;

/***************************************************************
 *                     CAN通信数据包
 ***************************************************************/
typedef struct{
    int16_t data1,data2,data3,data4;
}CAN_Structure;

typedef struct{
    pid inner;
    pid outer;
}PID_Structure;

/***************************************************************
 *                      底盘/云台结构体
 ***************************************************************/
typedef struct
{
    PID_Structure yaw_pid;
    PID_Structure Speed_X_PID, Speed_Y_PID;
    float inverse_speed_X,inverse_speed_Y;
    float forward_speed_X,forward_speed_Y;
    float target_omega,actual_omega;
    CAN_Structure Motor_1_RxData,Motor_2_RxData,Motor_3_RxData,Motor_4_RxData;

    //
    Low_Pass_Filter_Structure Speed_X_KF;
    Low_Pass_Filter_Structure Speed_Y_KF;
}CHASSIS;

typedef struct
{
    PID_Structure pitch_pid,yaw_pid;
    CAN_Structure pitch_RxData,yaw_RxData;
    SENDPACKET sendpacket;  // 用于向视觉发送
    SENDPACKET recvpacket;  // 接收视觉的数据
    SENDPACKET curr_angle;

    uint8_t sentry_state;
    Low_Pass_Filter_Structure gimbal_Receive_KF;
}GIMBAL;
/***************************************************************
 *                      主结构体
 ***************************************************************/
typedef struct{
    GIMBAL gimbal;
    CHASSIS chassis;

    CAN_Structure TxData;
    // 作为CAN收发的过程量
    //uint8_t MData[8];

    // 遥控器
    RC_Ctl_t rc_Data;

    // 用于各种测试
    float speed_cal[4];
}TYPEDEF;

//全局变量应包含这个结构体类型
#define SENTRY_MODE     2
#define TRACKING_MODE   3
#define GIMBAL_MODE     1
#define CHASSIS_MODE    1

/***************************************************************
 *
 ***************************************************************/
extern TYPEDEF reg;
extern TYPEDEF *p_reg;

extern osMessageQueueId_t USBRxQueueHandle;
extern osMessageQueueId_t RCQueueHandle;

#endif //GIMBAL_REG_H
