//
// Created by 18796 on 2026/4/17.
//
#include "bsp_bmi088.h"
#include <stdint.h>
#include "BMI088driver.h"
#include "reg.h"
#include "struct_typedef.h"

extern osThreadId_t bmi088TaskHandle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t receive_flag = 0;
    if (GPIO_Pin == INT1_GYRO_Pin) // 假设你用了 PC5（EXTI5）
    {
        BMI088_read(&p_reg->exti_bmi_data[0], NULL, NULL);
        receive_flag += 1;
    }
    if (GPIO_Pin == INT1_GYRO_Pin)
    {
        BMI088_read(NULL, &p_reg->exti_bmi_data[3], NULL);
        receive_flag += 1;
    }

    if (receive_flag == 2)
    {
        receive_flag = 0;
        p_reg->bmi_flag = 1;
    }
}
