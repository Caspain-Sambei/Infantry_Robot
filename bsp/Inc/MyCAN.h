//
// Created by 18796 on 2026/2/23.
//

#ifndef GIMBAL_MYCAN_H
#define GIMBAL_MYCAN_H

#include <stdint.h>
#include "reg.h"
#define hcanx       &hcan1

uint8_t CAN_bsp_Send(uint16_t ID,uint8_t *Data,uint8_t Length);
extern void CAN_Send(uint16_t ID,CAN_Structure *Data,uint8_t Length);
#endif //GIMBAL_MYCAN_H