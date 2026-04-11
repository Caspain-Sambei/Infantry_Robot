//
// Created by 18796 on 2026/2/27.
//
#include "bsp_rc.h"
#include <string.h>
#include "cmsis_os2.h"
#include "usart.h"
#include "reg.h"
#include "../../App/wheel/Omni_wheel.h"

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

    //your control code ….
    /****************************************************************
     *                 对全局变量进行赋值
     ****************************************************************/
    p_reg->rc_Data.rc.ch0 = RC_CtrlData.rc.ch0;
    p_reg->rc_Data.rc.ch1 = RC_CtrlData.rc.ch1;
    p_reg->rc_Data.rc.ch2 = RC_CtrlData.rc.ch2;
    p_reg->rc_Data.rc.ch3 = RC_CtrlData.rc.ch3;
    p_reg->rc_Data.rc.s1 = RC_CtrlData.rc.s1;
    p_reg->rc_Data.rc.s2 = RC_CtrlData.rc.s2;
    p_reg->rc_Data.mouse.x = RC_CtrlData.mouse.x;
    p_reg->rc_Data.mouse.y = RC_CtrlData.mouse.y;
    p_reg->rc_Data.mouse.z = RC_CtrlData.mouse.z;
    p_reg->rc_Data.mouse.press_l = RC_CtrlData.mouse.press_l;
    p_reg->rc_Data.mouse.press_r = RC_CtrlData.mouse.press_r;
    p_reg->rc_Data.key.v = RC_CtrlData.key.v;
}

//CT = 0: 当前使用Memory 0，下一个将使用Memory1。等于1则相反。
void UART3_DT7_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (huart3.hdmarx->Instance->CR &DMA_SxCR_CT)    // 判断当前DMA使用的是哪个内存缓冲区
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
        HAL_UART_DMAStop(&huart3);
        // 重新设置接收缓冲区为另一个缓冲
        huart3.hdmarx->Instance->PAR = (uint32_t)&huart3.Instance->DR; // 外设地址不变
        huart3.hdmarx->Instance->M0AR = (uint32_t)sbus_rx_buffer[1];   // 切换到另一个内存
        huart3.hdmarx->Instance->NDTR = RC_FRAME_LENGTH;               // 重置长度
        // 清除切换标志位，确保下一次从新的内存开始
        huart3.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
        // 重新使能DMA和串口空闲中断
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buffer[1], RC_FRAME_LENGTH);
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
        HAL_UART_DMAStop(&huart3);
        huart3.hdmarx->Instance->M0AR = (uint32_t)sbus_rx_buffer[0];
        huart3.hdmarx->Instance->NDTR = RC_FRAME_LENGTH;
        huart3.hdmarx->Instance->CR |= DMA_SxCR_CT; // 设置标志，表示下一次使用Memory1
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buffer[0], RC_FRAME_LENGTH);
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
         //     HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buffer[1], RC_FRAME_LENGTH);
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
        //     HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buffer[0], RC_FRAME_LENGTH);
        // }
//    }
//}