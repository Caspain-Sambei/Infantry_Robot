#include <string.h>
#include "bsp_Motor.h"
#include "reg.h"
#include "cmsis_os2.h"
#include "MyCAN.h"
#include "Omni_wheel.h"

/* USER CODE BEGIN Header_chassis_CAN_RxTask */
/**
* @brief Function implementing the chassis_CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_CAN_RxTask */
void chassis_CAN_RxTask(void *argument)
{
    /* USER CODE BEGIN chassisCAN_Rx */
    static uint8_t buf[9] = {0};
    /* Infinite loop */
    for (;;)
    {
        osMessageQueueGet(CAN_2RxQueueHandle, &buf,NULL,osWaitForever);

        if (buf[8] == 1)
        {
            p_reg->chassis.Motor_1_RxData.data1 = ((int16_t)buf[1] & 0xFF) | ((int16_t)buf[0] << 8);    // 转子机械角度
            p_reg->chassis.Motor_1_RxData.data2 = ((int16_t)buf[3] & 0xFF) | ((int16_t)buf[2] << 8);    // 转子转速
            p_reg->chassis.Motor_1_RxData.data3 = ((int16_t)buf[5] & 0xFF) | ((int16_t)buf[4] << 8);    // 实际扭矩电流
            p_reg->chassis.Motor_1_RxData.data4 = (int16_t)buf[6];                                      // 电机温度
        }
        if (buf[8] == 2)
        {
            p_reg->chassis.Motor_2_RxData.data1 = ((int16_t)buf[1] & 0xFF) | ((int16_t)buf[0] << 8);    // 转子机械角度
            p_reg->chassis.Motor_2_RxData.data2 = ((int16_t)buf[3] & 0xFF) | ((int16_t)buf[2] << 8);    // 转子转速
            p_reg->chassis.Motor_2_RxData.data3 = ((int16_t)buf[5] & 0xFF) | ((int16_t)buf[4] << 8);    // 实际扭矩电流
            p_reg->chassis.Motor_2_RxData.data4 = (int16_t)buf[6];                                      // 电机温度
        }
        if (buf[8] == 3)
        {
            p_reg->chassis.Motor_3_RxData.data1 = ((int16_t)buf[1] & 0xFF) | ((int16_t)buf[0] << 8);    // 转子机械角度
            p_reg->chassis.Motor_3_RxData.data2 = ((int16_t)buf[3] & 0xFF) | ((int16_t)buf[2] << 8);    // 转子转速
            p_reg->chassis.Motor_3_RxData.data3 = ((int16_t)buf[5] & 0xFF) | ((int16_t)buf[4] << 8);    // 实际扭矩电流
            p_reg->chassis.Motor_3_RxData.data4 = (int16_t)buf[6];                                      // 电机温度
        }
        if (buf[8] == 4)
        {
            p_reg->chassis.Motor_4_RxData.data1 = ((int16_t)buf[1] & 0xFF) | ((int16_t)buf[0] << 8);    // 转子机械角度
            p_reg->chassis.Motor_4_RxData.data2 = ((int16_t)buf[3] & 0xFF) | ((int16_t)buf[2] << 8);    // 转子转速
            p_reg->chassis.Motor_4_RxData.data3 = ((int16_t)buf[5] & 0xFF) | ((int16_t)buf[4] << 8);    // 实际扭矩电流
            p_reg->chassis.Motor_4_RxData.data4 = (int16_t)buf[6];                                      // 电机温度
        }
    }
    /* USER CODE END chassisCAN_Rx */
}

/***************************************************************************
 *								底盘的PID
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
        /************************************************************
         *                  底盘解算 + 单环PID
         ************************************************************/
        Omni_wheel_calculate(&p_reg->rc_Data,&p_reg->chassis,speed_cal);
        // 将结果映射为4个电机的电流
        p_reg->TxData.data1 = (int16_t)(speed_cal[0] * RC_TO_3508_Current);
        p_reg->TxData.data2 = (int16_t)(speed_cal[1] * RC_TO_3508_Current);
        p_reg->TxData.data3 = (int16_t)(speed_cal[2] * RC_TO_3508_Current);
        p_reg->TxData.data4 = (int16_t)(speed_cal[3] * RC_TO_3508_Current);


        CAN_Send(CAN_C620_1, &p_reg->TxData, 4);
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

        osDelay(1);
    }
    /* USER CODE END chassis_inPIDTask */
}
