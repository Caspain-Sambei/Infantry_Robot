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
#define EXT_ID         0x00
#define CAN_C620_1     0x200
#define CAN_C620_2     0x1FF

#define C620_ANGLE_RESOLUTION   0.0439f     // C620电调-电机机械角度-分辨率（单位：度/LSB）
#define CHASSIS_RADIUS          0.425f       // 单位：m
#define GEAR_RATIO_3508         15.764f     // 268.0f/17.0f区别于3508P19电机
#define PI                      3.1415926f

#define SPEED_TO_CURRENT        1.0f
// 3508:-16384~16384 RC:364~1024~1684
 #define RC_TO_3508_Current      24.824f     // 通道数值转化为电流值

#endif //GIMBAL_BSP_MOTOR_H