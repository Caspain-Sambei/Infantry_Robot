//
// Created by 18796 on 2026/2/23.
//
#include "MyCAN.h"
#include <string.h>
#include "can.h"
#include "bsp_Motor.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"

/**
 * @brief HAL_CAN_Start，使能接收中断，配置过滤器
 * @param hcanx CAN1~CAN2
 * @return HAL_CAN_Start初始化错误或者过滤器配置错误返回HAL_ERROR
 */
uint8_t CAN_Init(CAN_HandleTypeDef *hcanx)
{
    if (HAL_CAN_Start(hcanx) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_CAN_ActivateNotification(hcanx,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    if (hcanx == &hcan1)
    {
        CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(12) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        return HAL_OK;
    }
    if (hcanx == &hcan2)
    {
        CAN_Filter_Mask_Config(&hcan2, CAN_FILTER(15) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号[3:] | FIFOx[2:2] | ID类型[1:1] | 帧类型[0:0]
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x7ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    // 看第0位ID, 判断是数据帧还是遥控帧，遥控帧暂不处理
    if (Object_Para & 0x01)
        return;

    // 看第1位ID, 判断是标准帧还是扩展帧，扩展帧暂不处理
    if ((Object_Para & 0x02) >> 1)
        return;

    // 标准帧

    // ID配置, 标准帧的ID是11bit, 按规定放在高16bit中的[15:5]位
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ((ID & 0x7FF) << 5);;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = 0x0000;
    // 掩码后屏蔽位的高16bit
    can_filter_init_structure.FilterMaskIdHigh = ((Mask_ID & 0x7FF) << 5);
    // 掩码后屏蔽位的低16bit
    can_filter_init_structure.FilterMaskIdLow = 0x0000;

    // 滤波器配置
    // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
    can_filter_init_structure.FilterBank = (Object_Para >> 3) & 0x1F;
    // 滤波器模式, 设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    // 使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;

    // 从机模式配置
    // 从机模式选择开始单元, 一般均分14个单元给CAN1和CAN2
    can_filter_init_structure.SlaveStartFilterBank = 14;

    // 滤波器绑定FIFOx, 只能绑定一个
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 包含基础函数CAN_bsp_Send，适用CAN_C620_1(0x200),CAN_6020_1（0x1FF）
 * @param ID CAN_6020_1，CAN_6020_2，CAN_C620_1，CAN_C620_2
 * @param Data int16_t大小为4的数组
 * @param Length 4
 */
uint8_t CAN_Send(uint16_t ID,CAN_Structure *Data,uint8_t Length)
{
    uint8_t arr[8] = {0};

    arr[1] = (uint8_t)(Data->data1 & 0xFF);
    arr[0] = (uint8_t)(Data->data1 >> 8);
    arr[3] = (uint8_t)(Data->data2 & 0xFF);
    arr[2] = (uint8_t)(Data->data2 >> 8);
    arr[5] = (uint8_t)(Data->data3 & 0xFF);
    arr[4] = (uint8_t)(Data->data3 >> 8);
    arr[7] = (uint8_t)(Data->data4 & 0xFF);
    arr[6] = (uint8_t)(Data->data4 >> 8);

    if (ID == CAN_C620_1)
    {
        return CAN_bsp_Send(&hcan1,ID,arr,8);
    }
    if(ID == CAN_6020_1)
    {
        return CAN_bsp_Send(&hcan2,ID,arr,8);
    }
    return HAL_ERROR;
}

/**
 * @brief 用于CAN发送的基础函数
 * @param hcanx CAN编号
 * @param ID CAN_ID1或CAN_ID2
 * @param Data 类型为uint8_t
 * @param Length 数组大小
 * @return 发送成功即返回0,数值错误或者
 */
uint8_t CAN_bsp_Send(CAN_HandleTypeDef *hcanx,uint16_t ID,uint8_t *Data,uint8_t Length)
{
    if (Data == NULL || Length == 0 || Length > 8)
    {
        return HAL_ERROR;
    }
    // //检查CAN状态
    // if (HAL_CAN_GetState(hcanx)  != HAL_CAN_STATE_READY &&
    //    HAL_CAN_GetState(hcanx) != HAL_CAN_STATE_LISTENING)
    // {
    //     return HAL_ERROR;
    // }
    // 存储发送邮箱编号（HAL 需传入该变量接收邮箱号）
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef TxMessage = {0};

    TxMessage.StdId = ID;
    TxMessage.ExtId = 0;
    TxMessage.DLC = Length;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.TransmitGlobalTime = DISABLE; // 不发送时间戳

    return HAL_CAN_AddTxMessage(hcanx,&TxMessage,Data,&TxMailbox);
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
        uint8_t buf[8] = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, buf) == HAL_OK)
        {

            switch (RxHeader.StdId)
            {
                case (0x201):
                    {
                        p_reg->chassis.Motor_1_RxData.data1 = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
                        p_reg->chassis.Motor_1_RxData.data2 = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
                        p_reg->chassis.Motor_1_RxData.data3 = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
                        p_reg->chassis.Motor_1_RxData.data4 = (int16_t)buf[6];
                        break;
                    }
                case (0x202):
                    {
                        p_reg->chassis.Motor_2_RxData.data1 = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
                        p_reg->chassis.Motor_2_RxData.data2 = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
                        p_reg->chassis.Motor_2_RxData.data3 = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
                        p_reg->chassis.Motor_2_RxData.data4 = (int16_t)buf[6];
                        break;
                    }
                case (0x203):
                    {
                        p_reg->chassis.Motor_3_RxData.data1 = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
                        p_reg->chassis.Motor_3_RxData.data2 = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
                        p_reg->chassis.Motor_3_RxData.data3 = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
                        p_reg->chassis.Motor_3_RxData.data4 = (int16_t)buf[6];
                        break;
                    }
                case (0x204):
                    {
                        p_reg->chassis.Motor_4_RxData.data1 = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
                        p_reg->chassis.Motor_4_RxData.data2 = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
                        p_reg->chassis.Motor_4_RxData.data3 = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
                        p_reg->chassis.Motor_4_RxData.data4 = (int16_t)buf[6];
                        break;
                    }
            }
        }
    }
    /******************************************************
     *                  负责云台
     ******************************************************/
    if (hcan->Instance == CAN2)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t buf[8] = {0};

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, buf) == HAL_OK)
        {
            switch (RxHeader.StdId)
            {
            case (0x208):
                {
                    p_reg->gimbal.pitch_RxData.data1 = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
                    p_reg->gimbal.pitch_RxData.data2 = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
                    p_reg->gimbal.pitch_RxData.data3 = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
                    p_reg->gimbal.pitch_RxData.data4 = (int16_t)buf[6];
                    break;
                }
            case (0x206):
                {
                    p_reg->gimbal.yaw_RxData.data1 = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
                    p_reg->gimbal.yaw_RxData.data2 = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
                    p_reg->gimbal.yaw_RxData.data3 = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
                    p_reg->gimbal.yaw_RxData.data4 = (int16_t)buf[6];
                    break;
                }
            }
        }
    }
}
