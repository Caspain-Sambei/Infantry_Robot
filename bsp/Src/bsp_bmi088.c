//
// Created by 18796 on 2026/4/17.
//
#include <stdint.h>
#include "bsp_bmi088.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "reg.h"
#include "spi.h"

/* 原始数据缓冲区 */
static  uint8_t gyro_raw[8];   // 陀螺仪：ID(1) + 三轴数据(6) + 保留(1)
static  uint8_t accel_raw[6];  // 加速度计：三轴数据(6)
static  uint8_t accel_dma_buf[8]; // 多留1个字节给命令

/* DMA 完成标志 */
static  uint8_t gyro_dma_done = 0;   // 1 表示陀螺仪数据已就绪
static  uint8_t accel_dma_done = 0;  // 1 表示加速度计数据已就绪

/* 全局数据就绪标志（供 RTOS 任务轮询） */
volatile uint8_t bmi088_data_ready = 0;
static  uint8_t current_dma_sensor = 0; // 0: 无, 1: 陀螺仪, 2: 加速度计

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 【拦截防护】如果 SPI DMA 正在搬运，直接忽略本次中断，避免总线碰撞崩溃
    if (hspi1.State != HAL_SPI_STATE_READY)
    {
        return;
    }

    if (GPIO_Pin == INT1_GYRO_Pin)
    {
        current_dma_sensor = 1;
        BMI088_GYRO_NS_L();
        // 陀螺仪：发1字节命令，收6字节数据
        gyro_raw[0] = BMI088_GYRO_X_L | 0x80;
        HAL_SPI_TransmitReceive_DMA(&hspi1, gyro_raw, gyro_raw, 7);
    }
    else if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        current_dma_sensor = 2;
        BMI088_ACCEL_NS_L();
        // 加速度计：发1字节命令，收1字节哑 + 6字节数据
        accel_dma_buf[0] = BMI088_ACCEL_XOUT_L | 0x80;
        HAL_SPI_TransmitReceive_DMA(&hspi1, accel_dma_buf, accel_dma_buf, 8);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) return;

    if (current_dma_sensor == 1)
    {
        BMI088_GYRO_NS_H();
        current_dma_sensor = 0;
        // 解析原始数据为物理值
        BMI088_gyro_read_over(&gyro_raw[1], p_reg->bmi088_real_data.gyro);
        gyro_dma_done = 1;
    }
    else if (current_dma_sensor == 2)
    {
        BMI088_ACCEL_NS_H();
        current_dma_sensor = 0;
        // 解析原始数据为物理值
        BMI088_accel_read_over(&accel_raw[2], p_reg->bmi088_real_data.accel, NULL);
        accel_dma_done = 1;
    }

    if (gyro_dma_done && accel_dma_done)
    {
        bmi088_data_ready = 1;
        gyro_dma_done = 0;
        accel_dma_done = 0;
    }
}

/**
 * @brief  将加速度计切换到 SPI 模式，并读取两个传感器的 WHO_AM_I 寄存器
 * @param  accel_id   [out] 加速度计 ID（正确值应为 0x1E）
 * @param  gyro_id    [out] 陀螺仪 ID（正确值应为 0x0F）
 * @return uint8_t    0: 两个传感器 ID 均正确
 *                    1: 加速度计 ID 错误
 *                    2: 陀螺仪 ID 错误
 *                    3: 两个传感器 ID 均错误
 */
uint8_t BMI088_SwitchToSPI(uint8_t *accel_id, uint8_t *gyro_id)
{
    uint8_t ret = 0;
    volatile uint8_t dummy_rx;

    // ========== 1. 加速度计切换到 SPI 模式 ==========
    BMI088_ACCEL_NS_L();                              // 拉低片选
    uint8_t dummy_cmd = BMI088_ACC_CHIP_ID | 0x80;    // 读 ACC_CHIP_ID 命令
    HAL_SPI_Transmit(&hspi1, &dummy_cmd, 1, 10);      // 发送读命令
    HAL_SPI_Receive(&hspi1, (uint8_t *)&dummy_rx, 1, 10); // 接收数据（无效）
    BMI088_ACCEL_NS_H();                              // 拉高片选 → 上升沿切换到 SPI 模式

    // 等待传感器稳定
    BMI088_delay_ms(5);

    // ========== 2. 读取加速度计和陀螺仪的 ID ==========
    // *accel_id = BMI088_read_accel_who_am_i();
    // *gyro_id = BMI088_read_gyro_who_am_i();

    // ========== 3. 验证 ID 并返回结果 ==========
    if (*accel_id != BMI088_ACC_CHIP_ID_VALUE) {
        ret |= 0x01;
    }
    if (*gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        ret |= 0x02;
    }
    return ret;
}

/**
 * @brief 直接通过 SPI 读取陀螺仪单个寄存器值（不依赖现有宏）
 * @param reg  寄存器地址
 * @return uint8_t 寄存器值
 */
uint8_t BMI088_Gyro_ReadReg_Direct(uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0xFF}; // 读命令 + 哑字节
    uint8_t rx[2];

    BMI088_GYRO_NS_L();                         // 拉低片选
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    BMI088_GYRO_NS_H();                         // 拉高片选

    return rx[1];  // 第二个字节是有效数据（第一个是发送命令时的返回，忽略）
}

/**
 * @brief 验证陀螺仪数据就绪中断配置
 * @return uint8_t 0: 正确; 1: GYRO_CTRL错误; 2: INT_MAP错误; 3: 均错
 */
uint8_t BMI088_VerifyGyroInterrupt_Direct(void)
{
    uint16_t err = 0;
    uint16_t val;

    // 读取 GYRO_CTRL (0x15)
    val = BMI088_Gyro_ReadReg_Direct(BMI088_GYRO_CTRL);
    if (val != 0x80) {
        err |= 0x01;
        return err;
    }

    // 读取 INT3_INT4_IO_MAP (0x18)
    val = BMI088_Gyro_ReadReg_Direct(BMI088_GYRO_INT3_INT4_IO_MAP);
    if (val != 0x01) {
        err |= 0x02;
        return err;
    }

    return 0;
}
