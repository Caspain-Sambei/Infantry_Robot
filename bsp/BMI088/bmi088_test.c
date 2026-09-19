//
// Created by 18796 on 2026/4/23.
//

#include "BMI088driver.h"
#include <stdint.h>
#include <stdbool.h>
#include "BMI088Middleware.h"
#include "bsp_delay.h"

// ========== 用户需要实现的底层 SPI 函数 ==========
// 以下函数需根据你的硬件平台自行实现
extern uint8_t SPI_TransferByte(uint8_t tx_data);

// ========== BMI088 寄存器地址（仅列出用到的）==========
// 加速度计
#define BMI088_ACC_CHIP_ID         0x00
#define BMI088_ACC_SOFTRESET       0x7E
#define BMI088_ACC_PWR_CTRL        0x7D
#define BMI088_ACC_PWR_CONF        0x7C
#define BMI088_ACC_RANGE           0x41
#define BMI088_ACC_CONF            0x40
#define BMI088_INT1_IO_CONF        0x53
#define BMI088_INT2_IO_CONF        0x54
#define BMI088_INT_MAP_DATA        0x58

// 陀螺仪
#define BMI088_GYRO_CHIP_ID        0x00
#define BMI088_GYRO_SOFTRESET      0x14
#define BMI088_GYRO_RANGE          0x0F
#define BMI088_GYRO_BANDWIDTH      0x10
#define BMI088_GYRO_LPM1           0x11
#define BMI088_GYRO_INT_CTRL       0x15
#define BMI088_INT3_INT4_IO_CONF   0x16
#define BMI088_INT3_INT4_IO_MAP    0x18

// ========== 底层读写封装 ==========
static void BMI088_ACC_WriteReg(uint8_t reg, uint8_t value)
{
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg & 0x7F); // 写操作：最高位为 0
    BMI088_read_write_byte(value);
    BMI088_ACCEL_NS_H();
}

static uint8_t BMI088_ACC_ReadReg(uint8_t reg)
{
    uint8_t dummy, data;
    BMI088_ACCEL_NS_L();
    // SPI_TransferByte(reg | 0x80);   // 读操作：最高位为 1
    // dummy = SPI_TransferByte(0xFF); // 加速度计特有：第一字节为 Dummy
    // data  = SPI_TransferByte(0xFF); // 第二字节才是有效数据

    BMI088_read_write_byte(reg | 0x80);         // 读操作：最高位为 1
    dummy = BMI088_read_write_byte(0xFF);   // 加速度计特有：第一字节为 Dummy
    data = BMI088_read_write_byte(0xFF);    // 第二字节才是有效数据

    BMI088_ACCEL_NS_H();
    (void)dummy;
    return data;
}

static void BMI088_GYRO_WriteReg(uint8_t reg, uint8_t value)
{
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg & 0x7F); // 写操作：最高位为 0
    BMI088_read_write_byte(value);
    BMI088_GYRO_NS_H();
}

static uint8_t BMI088_GYRO_ReadReg(uint8_t reg)
{
    uint8_t data;
    BMI088_GYRO_NS_L();
    // SPI_TransferByte(reg | 0x80);
    // data = SPI_TransferByte(0xFF);  // 陀螺仪无 Dummy 字节

    BMI088_read_write_byte(reg | 0x80);
    data = BMI088_read_write_byte(0xFF);    // 陀螺仪无 Dummy 字节

    BMI088_GYRO_NS_H();
    return data;
}

// ========== 主初始化函数 ==========
/**
 * @brief 初始化 BMI088（SPI 模式，启用数据就绪中断）
 * @retval true  初始化成功，CHIP_ID 校验通过
 * @retval false 初始化失败，通信异常
 */
bool BMI088_Init(void)
{
    uint8_t id;

    // ---------- 步骤1：加速度计 SPI 模式唤醒 ----------
    // 根据手册第12页，加速度计上电后默认处于 I²C 模式，需通过 CSB1 的上升沿切换至 SPI。
    // 执行一次读操作（内容无效），CSB1 拉高后即锁定为 SPI 模式。
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(BMI088_ACC_CHIP_ID | 0x80);
    BMI088_read_write_byte(0xFF);   // dummy

    BMI088_ACCEL_NS_H();
    delay_ms(1);

    // ---------- 步骤2：软复位（可选，确保寄存器为默认值）----------
    // // 加速度计软复位
    // BMI088_ACC_WriteReg(BMI088_ACC_SOFTRESET, 0xB6);
    // delay_ms(1);  // 手册要求复位后等待 1ms
    //
    // // 陀螺仪软复位
    // BMI088_GYRO_WriteReg(BMI088_GYRO_SOFTRESET, 0xB6);
    // delay_ms(30); // 手册要求复位后等待 30ms

    // ---------- 步骤3：验证 CHIP_ID（可选）----------
    id = BMI088_ACC_ReadReg(BMI088_ACC_CHIP_ID);
    if (id != 0x1E) return false;

    id = BMI088_GYRO_ReadReg(BMI088_GYRO_CHIP_ID);
    if (id != 0x0F) return false;

    // ---------- 步骤4：加速度计配置 ----------
    // 4.1 设置量程 ±24g（手册5.3.9节）
    BMI088_ACC_WriteReg(BMI088_ACC_RANGE, 0x03);

    // 4.2 设置 ODR = 1600Hz，Normal 模式，保留 Bit7 = 1（手册5.3.8节）
    //     acc_odr = 0x0C (1600Hz)，acc_bwp = 0x02 (Normal)
    //     寄存器值 = (1<<7) | (0x02<<4) | 0x0C = 0xAC
    BMI088_ACC_WriteReg(BMI088_ACC_CONF, 0xAC);

    // 4.3 退出挂起模式，进入正常模式（手册4.1.1节）
    //     写 0x04 到 ACC_PWR_CTRL 使能加速度计
    BMI088_ACC_WriteReg(BMI088_ACC_PWR_CTRL, 0x04);
    delay_ms(50); // 手册要求切换模式后等待 50ms

    // 4.4 配置中断引脚 INT1：输出、推挽、高电平有效（手册5.3.10节）
    //     int1_out=1, int1_od=0(push-pull), int1_lvl=1(active high)
    BMI088_ACC_WriteReg(BMI088_INT1_IO_CONF, 0x0A);

    // 4.5 将加速度计数据就绪中断映射到 INT1（手册5.3.12节）
    //     Int1_drdy = 1 (bit2)
    BMI088_ACC_WriteReg(BMI088_INT_MAP_DATA, 0x04);

    // ---------- 步骤5：陀螺仪配置 ----------
    // 5.1 设置量程 ±2000°/s（手册5.5.4节）
    BMI088_GYRO_WriteReg(BMI088_GYRO_RANGE, 0x00);

    // 5.2 设置 ODR = 2kHz，滤波器带宽 532Hz（手册5.5.5节）
    BMI088_GYRO_WriteReg(BMI088_GYRO_BANDWIDTH, 0x00);

    // 5.3 确保退出低功耗模式（上电默认已是 Normal，此处可省略）
    BMI088_GYRO_WriteReg(BMI088_GYRO_LPM1, 0x00);

    // 5.4 使能新数据中断（手册4.7.2节）
    BMI088_GYRO_WriteReg(BMI088_GYRO_INT_CTRL, 0x80);

    // 5.5 配置中断引脚 INT3：输出、推挽、高电平有效（手册5.5.9节）
    //     Int3_od=0, Int3_lvl=1 -> 0x01
    BMI088_GYRO_WriteReg(BMI088_INT3_INT4_IO_CONF, 0x01);

    // 5.6 将数据就绪中断映射到 INT3（手册5.5.10节）
    BMI088_GYRO_WriteReg(BMI088_INT3_INT4_IO_MAP, 0x01);

    return true;
}
