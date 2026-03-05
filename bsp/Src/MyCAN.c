//
// Created by 18796 on 2026/2/23.
//
#include "MyCAN.h"
#include "reg.h"
#include "can.h"
#include "cmsis_os.h"
#include "bsp_Motor.h"

/**
 *
 * @param ID CAN_ID1或CAN_ID2
 * @param Data 类型为uint8_t
 * @param Length 数组大小
 * @return 发送成功即返回1
 */
uint8_t CAN_Send(uint16_t ID,uint8_t *Data,uint8_t Length)
{
    if (Data == NULL || Length == 0 || Length > 8)
    {
        return 0;
    }
    //检查CAN状态
    if (HAL_CAN_GetState(hcanx)  != HAL_CAN_STATE_READY &&
       HAL_CAN_GetState(hcanx) != HAL_CAN_STATE_LISTENING)
    {
        return 0;
    }
    // 存储发送邮箱编号（HAL 需传入该变量接收邮箱号）
    uint32_t TxMailbox;

    CAN_TxHeaderTypeDef TxMessage;
    TxMessage.StdId = ID;
    TxMessage.ExtId = EXT_ID;
    TxMessage.DLC = Length;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;

    HAL_StatusTypeDef ret =
    HAL_CAN_AddTxMessage(hcanx,&TxMessage,Data,&TxMailbox);

    return (ret == HAL_OK)? 1:0;
}

/**
 * @brief 可接受CAN1 ID1~4的电机反馈报文
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t buf[9] = {0};

        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, buf) == HAL_OK)
        {
            switch (RxHeader.StdId)
            {
                case (0x201):
                    {
                        buf[8] = 1;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                        break;
                    }
                case (0x202):
                    {
                        buf[8] = 2;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                        break;
                    }
                case (0x203):
                    {
                        buf[8] = 3;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                        break;
                    }
                case (0x204):
                    {
                        buf[8] = 4;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                        break;
                    }
                case (0x205):
                    {
                        buf[8] = 5;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                    }
                case (0x206):
                    {
                        buf[8] = 6;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                    }
                case (0x207):
                    {
                        buf[8] = 7;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                    }
                case (0x208):
                    {
                        buf[8] = 8;
                        osMessageQueuePut(CAN_1RxQueueHandle, buf, 0, 0);
                    }
            }
        }
    }
    if (hcan->Instance == CAN2)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t buf[9] = {0};

        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, buf) == HAL_OK)
        {
            switch (RxHeader.StdId)
            {
            case (0x201):
                {
                    buf[8] = 1;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                    break;
                }
            case (0x202):
                {
                    buf[8] = 2;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                    break;
                }
            case (0x203):
                {
                    buf[8] = 3;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                    break;
                }
            case (0x204):
                {
                    buf[8] = 4;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                    break;
                }
            case (0x205):
                {
                    buf[8] = 5;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                }
            case (0x206):
                {
                    buf[8] = 6;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                }
            case (0x207):
                {
                    buf[8] = 7;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                }
            case (0x208):
                {
                    buf[8] = 8;
                    osMessageQueuePut(CAN_2RxQueueHandle, buf, 0, 0);
                }
            }
        }
    }
}