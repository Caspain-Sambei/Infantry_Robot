//
// Created by 18796 on 2026/2/23.
//

#ifndef GIMBAL_MYCAN_H
#define GIMBAL_MYCAN_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "reg.h"
#include "stm32f4xx_hal_can.h"

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

uint8_t CAN_bsp_Send(CAN_HandleTypeDef *hcanx,uint16_t ID,uint8_t *Data,uint8_t Length);
extern uint8_t CAN_Send(uint16_t ID,CAN_Structure *Data,uint8_t Length);
uint8_t CAN_Init(CAN_HandleTypeDef *hcanx);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

#endif //GIMBAL_MYCAN_H
