//
// Created by 18796 on 2026/2/27.
//
/* USER CODE END Header_StartSbusProcessor */
#include <string.h>
#include "bsp_Motor.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "bsp_rc.h"
#include "reg.h"

/********************************************************
* 以轮子逆时针方向为正转
* 左上为1，右上为2，左下为3，右下为4
 ********************************************************/

void StartSbusTransTask(void *argument)
{
    /* USER CODE BEGIN StartSbusProcessor */
    uint8_t RxData[RC_FRAME_LENGTH] = {0};
    /* Infinite loop */
    for(;;)
    {
        osMessageQueueGet(SbusFrameQueueHandle,RxData,0,osWaitForever);
        RemoteDataProcess(RxData);
        osDelay(1);
    }
    /* USER CODE END StartSbusProcessor */
}

/* USER CODE END Header_StartSubsProcess */
void StartSubsProcessTask(void *argument)
{
    /* USER CODE BEGIN StartSubsProcess */
    /* Infinite loop */
    for(;;)
    {
        osMessageQueueGet(Sbus_2ndQueueHandle,&p_reg->rc_Data,0,osWaitForever);

        float left_right = (float)p_reg->rc_Data.rc.ch2 * RC_TO_3508_Current * 0.01f;
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

            Motor_Send(CAN_1,&p_reg->TxData,4);
            // 每次发送完清零
            memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
        }
        if (left_right != 0.0f)
        {
            if (left_right > 0.0f)      // 向右
            {
                p_reg->TxData.data1 = (int16_t)left_right;
                p_reg->TxData.data2 = (int16_t)left_right;
                p_reg->TxData.data3 = (int16_t)-left_right;
                p_reg->TxData.data4 = (int16_t)-left_right;

                Motor_Send(CAN_1,&p_reg->TxData,4);
                // 每次发送完清零
                memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
            }
            else
            {
                p_reg->TxData.data1 = (int16_t)-left_right;
                p_reg->TxData.data2 = (int16_t)-left_right;
                p_reg->TxData.data3 = (int16_t)left_right;
                p_reg->TxData.data4 = (int16_t)left_right;

                Motor_Send(CAN_1,&p_reg->TxData,4);
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

            Motor_Send(CAN_1,&p_reg->TxData,4);
            // 每次发送完清零
            memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
        }
        else
        {
            p_reg->TxData.data1 = (int16_t)turn;
            p_reg->TxData.data2 = (int16_t)turn;
            p_reg->TxData.data3 = (int16_t)turn;
            p_reg->TxData.data4 = (int16_t)turn;

            Motor_Send(CAN_1,&p_reg->TxData,4);
            // 每次发送完清零
            memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
        }

    }
    /* USER CODE END StartSubsProcess */
}
