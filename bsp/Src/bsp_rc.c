//
// Created by 18796 on 2026/2/27.
//
#include "bsp_rc.h"
#include <string.h>
#include "bsp_Motor.h"
#include "cmsis_os2.h"
#include "MyCAN.h"
#include "usart.h"
#include "reg.h"

/* ----------------------- Internal Data ----------------------------------- */
volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];  //double sbus rx buffer to save data
static RC_Ctl_t RC_CtrlData;

/******************************************************************************
 * @fn      RemoteDataProcess
 *
 * @brief   resolution rc protocol data.
 * @pData   a point to rc receive buffer.
 * @return  None.
 * @note    RC_CtrlData is a global variable.you can deal with it in other place.
 */
void RemoteDataProcess(uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }

    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                         ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;

    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);

    /****************************************************************
     *                      my code
     *                      全向轮
     *                      1~4：左上，右上，左下，右下
     ****************************************************************/
    float left_right = (float)p_reg->rc_Data.rc.ch2 * RC_TO_3508_Current * 0.01f;
    p_reg->chassis.target_speed = left_right;
    float forward_back = (float)p_reg->rc_Data.rc.ch3 * RC_TO_3508_Current * 0.01f;
    // 水平转动先给个相对小值
    float turn = (float)p_reg->rc_Data.rc.ch0 * RC_TO_3508_Current * 0.01f;
    //float push = (float)p_reg->rc_Data.rc.ch1 * RC_TO_3508_Current;
    /****************************************************************
     *                      水平平动
     ****************************************************************/
    if (forward_back != 0.0f)
    {
        p_reg->TxData.data1 = (int16_t)forward_back;
        p_reg->TxData.data2 = (int16_t)-forward_back;
        p_reg->TxData.data3 = (int16_t)forward_back;
        p_reg->TxData.data4 = (int16_t)-forward_back;

        CAN_Send(CAN_C620_1, &p_reg->TxData, 4);
        // 每次发送完清零
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
    }
    if (left_right != 0.0f)
    {
        if (left_right > 0.0f) // 向右
        {
            p_reg->TxData.data1 = (int16_t)left_right;
            p_reg->TxData.data2 = (int16_t)left_right;
            p_reg->TxData.data3 = (int16_t)-left_right;
            p_reg->TxData.data4 = (int16_t)-left_right;

            CAN_Send(CAN_C620_1, &p_reg->TxData, 4);
            // 每次发送完清零
            memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
        }
        else
        {
            p_reg->TxData.data1 = (int16_t)-left_right;
            p_reg->TxData.data2 = (int16_t)-left_right;
            p_reg->TxData.data3 = (int16_t)left_right;
            p_reg->TxData.data4 = (int16_t)left_right;

            CAN_Send(CAN_C620_1, &p_reg->TxData, 4);
            // 每次发送完清零
            memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
        }
    }
    /****************************************************************
     *                      水平转动;设遥杆向右为正
     ****************************************************************/
    if (turn > 0.0f)
    {
        p_reg->TxData.data1 = (int16_t)turn;
        p_reg->TxData.data2 = (int16_t)turn;
        p_reg->TxData.data3 = (int16_t)turn;
        p_reg->TxData.data4 = (int16_t)turn;

        CAN_Send(CAN_C620_1, &p_reg->TxData, 4);
        // 每次发送完清零
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
    }
    else
    {
        p_reg->TxData.data1 = (int16_t)turn;
        p_reg->TxData.data2 = (int16_t)turn;
        p_reg->TxData.data3 = (int16_t)turn;
        p_reg->TxData.data4 = (int16_t)turn;

        CAN_Send(CAN_C620_1, &p_reg->TxData, 4);
        // 每次发送完清零
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
    }

}

//CT = 0: 当前使用Memory 0，下一个将使用Memory1。等于1则相反。
void UART5_DT7_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (huart5.hdmarx->Instance->CR &DMA_SxCR_CT)    // 判断当前DMA使用的是哪个内存缓冲区
    {
        // 当前使用的是 Memory 1 (缓冲区1)，那么收到数据的是缓冲区0，应处理缓冲区0的数据
        //RemoteDataProcess(sbus_rx_buffer[0]);
        uint8_t temp[RC_FRAME_LENGTH];
        // 手动拷贝，明确指定volatile源
        for (int i = 0; i < RC_FRAME_LENGTH; i++)
        {
            temp[i] = sbus_rx_buffer[0][i]; // 这里受volatile保护
        }
        osMessageQueuePut(SbusFrameQueueHandle,temp,0,0);

        // 准备下一次接收：重新配置DMA目标为 缓冲区1
        // 先停止DMA（HAL提供了原子操作）
        HAL_UART_DMAStop(&huart5);
        // 重新设置接收缓冲区为另一个缓冲
        huart5.hdmarx->Instance->PAR = (uint32_t)&huart5.Instance->DR; // 外设地址不变
        huart5.hdmarx->Instance->M0AR = (uint32_t)sbus_rx_buffer[1];   // 切换到另一个内存
        huart5.hdmarx->Instance->NDTR = RC_FRAME_LENGTH;               // 重置长度
        // 清除切换标志位，确保下一次从新的内存开始
        huart5.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
        // 重新使能DMA和串口空闲中断
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buffer[1], RC_FRAME_LENGTH);
    }
    else
    {
        // 当前使用的是 Memory 0，则处理缓冲区1的数据
        //RemoteDataProcess(sbus_rx_buffer[1]);

        uint8_t temp[RC_FRAME_LENGTH];
        // 手动拷贝，明确指定volatile源
        for (int i = 0; i < RC_FRAME_LENGTH; i++)
        {
            temp[i] = sbus_rx_buffer[1][i]; // 这里受volatile保护
        }
        osMessageQueuePut(SbusFrameQueueHandle,temp,0,0);

        // 切换回缓冲区0
        HAL_UART_DMAStop(&huart5);
        huart5.hdmarx->Instance->M0AR = (uint32_t)sbus_rx_buffer[0];
        huart5.hdmarx->Instance->NDTR = RC_FRAME_LENGTH;
        huart5.hdmarx->Instance->CR |= DMA_SxCR_CT; // 设置标志，表示下一次使用Memory1
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buffer[0], RC_FRAME_LENGTH);
    }
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
//{
//    if (huart->Instance == USART2)
//    {
         // if (huart->hdmarx->Instance->CR &DMA_SxCR_CT)    // 判断当前DMA使用的是哪个内存缓冲区
         // {
         //     // 当前使用的是 Memory 1 (缓冲区1)，那么收到数据的是缓冲区0，应处理缓冲区0的数据
         //     //RemoteDataProcess(sbus_rx_buffer[0]);
         //     uint8_t temp[RC_FRAME_LENGTH];
         //         // 手动拷贝，明确指定volatile源
         //     for (int i = 0; i < RC_FRAME_LENGTH; i++)
         //     {
         //          temp[i] = sbus_rx_buffer[0][i]; // 这里受volatile保护
         //      }
         //      osMessageQueuePut(&SbusFrameQueue,temp,0,0);
         //
         //      // 准备下一次接收：重新配置DMA目标为 缓冲区1
         //      // 先停止DMA（HAL提供了原子操作）
         //     HAL_UART_DMAStop(huart);
         //     // 重新设置接收缓冲区为另一个缓冲
         //     huart->hdmarx->Instance->PAR = (uint32_t)&huart->Instance->DR; // 外设地址不变
         //     huart->hdmarx->Instance->M0AR = (uint32_t)sbus_rx_buffer[1];   // 切换到另一个内存
         //     huart->hdmarx->Instance->NDTR = RC_FRAME_LENGTH;               // 重置长度
         //     // 清除切换标志位，确保下一次从新的内存开始
         //     huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
         //     // 重新使能DMA和串口空闲中断
         //     HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buffer[1], RC_FRAME_LENGTH);
         // }
        // else
        //  {
        //      // 当前使用的是 Memory 0，则处理缓冲区1的数据
        //      //RemoteDataProcess(sbus_rx_buffer[1]);
        //
        //     uint8_t temp[RC_FRAME_LENGTH];
        //     // 手动拷贝，明确指定volatile源
        //     for (int i = 0; i < RC_FRAME_LENGTH; i++)
        //     {
        //         temp[i] = sbus_rx_buffer[1][i]; // 这里受volatile保护
        //     }
        //     osMessageQueuePut(&SbusFrameQueue,temp,0,0);
        //
        //     // 切换回缓冲区0
        //     HAL_UART_DMAStop(huart);
        //     huart->hdmarx->Instance->M0AR = (uint32_t)sbus_rx_buffer[0];
        //     huart->hdmarx->Instance->NDTR = RC_FRAME_LENGTH;
        //     huart->hdmarx->Instance->CR |= DMA_SxCR_CT; // 设置标志，表示下一次使用Memory1
        //     HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buffer[0], RC_FRAME_LENGTH);
        // }
//    }
//}