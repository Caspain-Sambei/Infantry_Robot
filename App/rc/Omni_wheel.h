//
// Created by 18796 on 2026/3/28.
//

#ifndef GIMBAL_OMNI_WHEEL_H
#define GIMBAL_OMNI_WHEEL_H

#include "stm32f4xx_hal.h"
#include "reg.h"
#include "bsp_Motor.h"
#include "MyCAN.h"
#include "bsp_Motor.h"
#include <string.h>
#include "PID.h"
#include <math.h>

void Omni_wheel_calculate(const RC_Ctl_t *rc_Data,CHASSIS *chassis,float speed_cal[4]);

#endif //GIMBAL_OMNI_WHEEL_H