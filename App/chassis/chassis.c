#include <string.h>
#include "bsp_Motor.h"
#include "reg.h"
#include "cmsis_os2.h"
#include "MyCAN.h"
#include "../wheel/Omni_wheel.h"

/***************************************************************************
 *								底盘解算 + 单环PID
 **************************************************************************/
/* USER CODE BEGIN Header_chassis_inPIDTask */
/**
* @brief Function implementing the chassis_inPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_inPIDTask */
void chassis_inPIDTask(void *argument)
{
    /* USER CODE BEGIN chassis_inPIDTask */
    float speed_cal[4] = {0};
    /* Infinite loop */
    for(;;)
    {
        Omni_wheel_calculate(&p_reg->rc_Data,&p_reg->chassis,p_reg->speed_cal);
        // 将结果映射为4个电机的电流
        p_reg->TxData.data1 = (int16_t)(p_reg->speed_cal[0] * RC_TO_3508_Current);
        p_reg->TxData.data2 = (int16_t)(p_reg->speed_cal[1] * RC_TO_3508_Current);
        p_reg->TxData.data3 = (int16_t)(p_reg->speed_cal[2] * RC_TO_3508_Current);
        p_reg->TxData.data4 = (int16_t)(p_reg->speed_cal[3] * RC_TO_3508_Current);

        CAN_Send(CAN_C620_1, &p_reg->TxData, 4);
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

        osDelay(1);
    }
    /* USER CODE END chassis_inPIDTask */
}
