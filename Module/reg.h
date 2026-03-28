//
// Created by 18796 on 2026/2/23.
//

#ifndef GIMBAL_REG_H
#define GIMBAL_REG_H
#include <stdint.h>
/***************************************************************
 *                     遥控器通信
 ***************************************************************/
#include "bsp_rc.h"

/***************************************************************
 *                     视觉通信
 ***************************************************************/
typedef struct
{
    float yaw;
    float pitch;
    float roll;
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
    PID_Structure pitch_pid,yaw_pid,roll_pid;
    PID_Structure Speed_pid;
    float speed_X,speed_Y;
    float target_omega,actual_omega;
    CAN_Structure Motor_1_RxData,Motor_2_RxData,Motor_3_RxData,Motor_4_RxData;
}CHASSIS;

typedef struct
{
    PID_Structure pitch_pid,yaw_pid;
    CAN_Structure pitch_RxData,yaw_RxData;
    SENDPACKET sendpacket;  // 用于向视觉发送
    SENDPACKET recvpacket;  // 接收视觉的数据
    SENDPACKET curr_angle;

    uint8_t sentry_state;
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
}TYPEDEF;

/***************************************************************
 *
 ***************************************************************/
extern TYPEDEF reg;
extern TYPEDEF *p_reg;

extern osMessageQueueId_t USBRxQueueHandle;
extern osMessageQueueId_t CAN_2RxQueueHandle;
extern osMessageQueueId_t SbusFrameQueueHandle;

#endif //GIMBAL_REG_H
