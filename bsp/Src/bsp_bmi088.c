//
// Created by 18796 on 2026/4/17.
//
#include "bsp_bmi088.h"
#include <stdint.h>
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include "reg.h"
#include "spi.h"

static volatile uint8_t gyro_flag = 0;
static volatile uint8_t accel_flag = 0;
static volatile uint8_t gyro_raw[8];   // 陀螺仪原始数据（ID + 三轴）
static volatile uint8_t accel_raw[6];  // 加速度计原始数据（三轴）

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_GYRO_Pin)  // 假设 INT1 和 INT3 已短接，共用此引脚
    {
        // 拉低陀螺仪片选
        BMI088_GYRO_NS_L();

        // 发送读命令（从 GYRO_CHIP_ID 寄存器开始读 8 字节）
        uint8_t cmd = BMI088_GYRO_X_L | 0x80;
        HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);  // 发送 1 字节命令（阻塞，耗时极短）

        // 启动 DMA 接收 8 字节数据
        HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)gyro_raw, 8);
        gyro_flag = 1;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        if (gyro_flag == 1)
        {
            // 陀螺仪 DMA 接收完成
            BMI088_GYRO_NS_H();  // 拉高片选
            gyro_flag = 2;

            // 解析陀螺仪数据（使用你驱动中的解析函数）
            BMI088_gyro_read_over((uint8_t *)gyro_raw, &p_reg->bmi088_real_data.gyro[0]);

            // 启动加速度计 DMA 读取
            BMI088_ACCEL_NS_L();
            uint8_t cmd = BMI088_ACCEL_XOUT_L | 0x80;
            HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
            HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)accel_raw, 6);
        }
        else
        {
            // 加速度计 DMA 接收完成
            BMI088_ACCEL_NS_H();

            // 解析加速度计数据
            BMI088_accel_read_over((uint8_t *)accel_raw, &p_reg->bmi088_real_data.accel[0], NULL);

            // 两个传感器数据均已就绪，设置标志位
            p_reg->bmi_flag = 1;
            // 重置状态，准备下一次触发
            gyro_flag = 0;
        }
    }
}