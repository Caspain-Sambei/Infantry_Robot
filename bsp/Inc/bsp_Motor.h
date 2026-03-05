//
// Created by 18796 on 2026/3/1.
//

#ifndef GIMBAL_BSP_MOTOR_H
#define GIMBAL_BSP_MOTOR_H
#include <stdint.h>
#include "reg.h"
enum Enum_CAN_Motor_Status
{
    CAN_Motor_DISABLE = 0,
    CAN_Motor_Enable = 1,
};

#define CAN_6020_1     0x1FE
#define CAN_6020_2     0x2FE
#define EXT_ID     0
// 3508:-16384~16384 RC:364~1024~1684
#define RC_TO_3508_Current      24.824f     // 通道数值转化为电流值
#define C620_ANGLE_RESOLUTION   0.0439f     // C620电调-电机机械角度-分辨率（单位：度/LSB）
#define CHASSIS_RADIUS          0.37054     // 单位：m，由长宽529mm 519mm算得轮中心距底盘中心距离

extern void Motor_Send(uint16_t ID,CAN_Structure *Data,uint8_t Length);

#endif //GIMBAL_BSP_MOTOR_H