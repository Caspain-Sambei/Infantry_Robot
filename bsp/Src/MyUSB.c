//
// Created by 18796 on 2026/2/24.
//
#include <string.h>
#include "MyUSB.h"
#include <stdlib.h>
#include "cmsis_os2.h"
#include "reg.h"
#include "usbd_cdc_if.h"

extern osMessageQueueId_t USBRxQueueHandle;

// 把 float 类型的成员，通过指针重解释，转换为 uint8_t 字节数组。
// 指针重解释：改变访问内存的“浓度”，float是连续4个字节，uint8_t是一个字节

/**
 *
 * @param Data SENPACKET类型的结构体成员
 */
void USB_Send(SENDPACKET *Data)
{
    static uint8_t buf[3 * 4 + 2] = {0};
    buf[0] = USB_HEAD;
    buf[13] = USB_TAIL;
    uint8_t *p_byte;

    // Data->pitch去地址后强制转换类型
    // Data->pitch已经是指针所指向的那个结构体成员了？而不是指向那个成员的指针
    p_byte = (uint8_t*)&Data->pitch;
    buf[1] = p_byte[0];
    buf[2] = p_byte[1];
    buf[3] = p_byte[2];
    buf[4] = p_byte[3];

    p_byte = (uint8_t*)&Data->yaw;
    buf[5] = p_byte[0];
    buf[6] = p_byte[1];
    buf[7] = p_byte[2];
    buf[8] = p_byte[3];

    p_byte = (uint8_t*)&Data->roll;
    buf[9] = p_byte[0];
    buf[10] = p_byte[1];
    buf[11] = p_byte[2];
    buf[12] = p_byte[3];

    CDC_Transmit_FS(buf,sizeof(buf));
}

// uint8_t CDC_Receive_FS(uint8_t* Data, uint32_t Len)
// {
//     uint8_t STD_LEN = USB_FRAME_LEN; //固定帧长度
//     static uint8_t buf[USB_FRAME_LEN] = {0};
//     static uint8_t rx_cnt = 0;
//     uint8_t RxFlag = 0;
//
//     // 入参保护：空指针/长度0直接返回
//     if (Data == NULL || Len == 0)
//     {
//         return 0;
//     }
//
//      for (uint8_t i = 0; i < Len; i++)
//      {
//         if (rx_cnt == 0) //收到帧头才开始 接收数据
//         {
//             if (Data[i] == USB_RX_HEAD)
//             {
//                 buf[rx_cnt ++] = Data[i];
//             }
//         }
//         else // 收到帧头，接受后续数据
//         {
//             buf[rx_cnt ++] = Data[i];
//             if (rx_cnt >= STD_LEN)
//             {
//                 if (buf[rx_cnt - 1] == USB_RX_TAIL)
//                  {
//                     rx_cnt = 0;
//                     RxFlag = 1;
//                 }
//                 else
//                 {
//                     rx_cnt = 0;
//                     memset(buf,0,sizeof(buf));
//                 }
//             }
//         }
//     }
//     // 正确接收到数据后用队列传递到处理数据函数
//     if (RxFlag == 1)
//     {
//         RxFlag = 0;
//         osMessageQueuePut(USBRxQueueHandle,buf,0,0);
//     }
//     return Len;
// }