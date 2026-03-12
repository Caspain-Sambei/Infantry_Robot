#include <string.h>
#include <tgmath.h>
#include "bsp_Motor.h"
#include "reg.h"
#include "cmsis_os2.h"
#include "MyCAN.h"
#include "PID.h"

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
        // 获取底盘速度
        p_reg->chassis.actual_speed = (p_reg->chassis.Motor_1_RxData.data2 *sinf(45)
                                    +p_reg->chassis.Motor_1_RxData.data2 *sinf(45)
                                    +p_reg->chassis.Motor_1_RxData.data2 *sinf(45)
                                    +p_reg->chassis.Motor_1_RxData.data2 *sinf(45)) / 4.0f;
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
    /* Infinite loop */
    for(;;)
    {
        /************************************************************
         *                  PID
         ************************************************************/

        p_reg->chassis.Speed_pid.inner.Target = p_reg->chassis.target_speed;
        p_reg->chassis.Speed_pid.inner.Actual = p_reg->chassis.actual_speed;

        PID_Update(&p_reg->chassis.Speed_pid.inner,chassis_mode);

        /************************************************************
         *                  底盘速度解算
         ************************************************************/
        p_reg->chassis.yaw_pid.outer.Actual = p_reg->gimbal.yaw_pid.outer.Actual;
        float sin_ang = sinf(p_reg->chassis.yaw_pid.outer.Actual);
        float cos_ang = cosf(p_reg->chassis.yaw_pid.outer.Actual);
        float speed_X = p_reg->chassis.Speed_pid.inner.Out * cosf(p_reg->chassis.target_angle);
        float speed_Y = p_reg->chassis.Speed_pid.inner.Out * sinf(p_reg->chassis.target_angle);

        float speed_cal[4] = {0.0f};

        speed_cal[0] = (float)((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);
        speed_cal[1] = (float)((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);
        speed_cal[2] = (float)((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);
        speed_cal[3] = (float)((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);

        /************************************************************
         *                   CAN发送
         ************************************************************/
        // 将结果转变为4个电机的转速
        p_reg->TxData.data1 = (int16_t)speed_cal[0];
        p_reg->TxData.data2 = (int16_t)speed_cal[1];
        p_reg->TxData.data3 = (int16_t)speed_cal[2];
        p_reg->TxData.data4 = (int16_t)speed_cal[3];

        CAN_Send(CAN_C620_1,&p_reg->TxData,4);
        // 每次发送完清零
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

        osDelay(1);
    }
    /* USER CODE END chassis_inPIDTask */
}
