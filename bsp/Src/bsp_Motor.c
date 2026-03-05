

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "bsp_rc.h"
#include "reg.h"
#include "bsp_Motor.h"
#include <string.h>
#include "MyCAN.h"

/********************************************************
 *                      工具函数
 ********************************************************/

void Motor_Send(uint16_t ID,CAN_Structure *Data,uint8_t Length)
{
    uint8_t arr[8] = {0};

    arr[0] = (uint8_t)(Data->data1 & 0xFF);
    arr[1] = (uint8_t)(Data->data1 >> 8);
    arr[2] = (uint8_t)(Data->data2 & 0xFF);
    arr[3] = (uint8_t)(Data->data2 >> 8);
    arr[4] = (uint8_t)(Data->data3 & 0xFF);
    arr[5] = (uint8_t)(Data->data3 >> 8);
    arr[6] = (uint8_t)(Data->data4 & 0xFF);
    arr[7] = (uint8_t)(Data->data4 >> 8);

    CAN_Send(CAN_2,arr,8);
}