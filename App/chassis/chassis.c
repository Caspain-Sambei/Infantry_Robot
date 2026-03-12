#include <tgmath.h>
#include "bsp_Motor.h"
#include "reg.h"
#include "cmsis_os2.h"

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

/* USER CODE BEGIN Header_chassis_calculateTask */
/**
* @brief Function implementing the chassis_calcul thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_calculateTask */
void chassis_calculateTask(void *argument)
{
    /* USER CODE BEGIN chassis_calculate */
    float speed_cal[4] = {0.0f};
    int speed_out[4] = {0};

    /* Infinite loop */
    for(;;)
    {
        p_reg->chassis.yaw_pid.outer.Actual = p_reg->gimbal.yaw_pid.outer.Actual;
        float sin_ang = sinf(p_reg->chassis.yaw_pid.outer.Actual);
        float cos_ang = cosf(p_reg->chassis.yaw_pid.outer.Actual);
        float speed_X = p_reg->chassis.target_speed * cosf(p_reg->chassis.target_angle);
        float speed_Y = p_reg->chassis.target_speed * sinf(p_reg->chassis.target_angle);

        speed_cal[0] = (float)((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);
        speed_cal[1] = (float)((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);
        speed_cal[2] = (float)((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);
        speed_cal[3] = (float)((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + CHASSIS_RADIUS  * p_reg->chassis.target_omega)
                        / sqrtf(2);
    }
    /* USER CODE END chassis_calculate */
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
        osDelay(1);
    }
    /* USER CODE END chassis_inPIDTask */
}
