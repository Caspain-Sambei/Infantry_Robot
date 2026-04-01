//
// Created by 18796 on 2026/2/27.
//
/* USER CODE END Header_StartSbusProcessor */
#include <string.h>
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "bsp_rc.h"
#include "MyCAN.h"
#include "reg.h"

/********************************************************
* 以轮子逆时针方向为正转
* 左上为1，右上为2，左下为3，右下为4
 ********************************************************/
/* USER CODE BEGIN Header_StartSbusTransTask */
/**
* @brief Function implementing the SbusTransTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSbusTransTask */
void StartSbusTransTask(void *argument)
{
    /* USER CODE BEGIN StartSbusProcessor */
    uint8_t RxData[RC_FRAME_LENGTH] = {0};
    /* Infinite loop */
    for(;;)
    {
        osMessageQueueGet(SbusFrameQueueHandle,RxData,0,osWaitForever);
        RemoteDataProcess(RxData);
        osDelay(10);
    }
    /* USER CODE END StartSbusProcessor */
}
