#include "bsp_delay.h"

#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS.h"
static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

void delay_init(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;
}

// void delay_us(uint16_t nus)
// {
//     uint32_t ticks = 0;     // 目标延时需要的总Ticks数
//     uint32_t told = 0;      // 上一次读取的SysTick计数值
//     uint32_t tnow = 0;      // 当前读取的SysTick计数值
//     uint32_t tcnt = 0;      // 已经累计的Ticks数
//     uint32_t reload = 0;    // SysTick的重装值（最大计数值）
//     reload = SysTick->LOAD;
//     ticks = nus * fac_us;   // 计算目标延时nus微秒需要的总Ticks数
//     told = SysTick->VAL;
//     // SysTick 是向下递减且自动重载的，71999->71998->……
//     while (1)
//     {
//         tnow = SysTick->VAL;
//         if (tnow != told)   // 只有计数值变化了，才累计耗时
//         {
//             if (tnow < told)
//             {
//                 // 情况1：定时器未溢出（tnow < told），直接减
//                 tcnt += told - tnow;
//             }
//             else
//             {
//                 // 情况2：定时器溢出（tnow > told），需加上重载值补全
//                 tcnt += reload - tnow + told;
//             }
//             told = tnow;    // 更新基准值为当前值，下次循环对比
//             if (tcnt >= ticks)  // 计数值达到目标微秒，退出
//             {
//                 break;
//             }
//         }
//     }
// }

void delay_us(uint16_t nus)
{
    // 入参保护
    if (nus == 0 || nus >= 1000) return;

    uint32_t ticks = nus * fac_us;
    uint32_t start_tick = SysTick->VAL;

    // 忙等延时（短时间占用CPU，对系统影响小）
    // while ((start_tick - SysTick->VAL) < ticks)
    // {
    //     __NOP(); // 空操作，减少CPU消耗
    // }
    if (start_tick < ticks)
    {
        while (SysTick->VAL > (start_tick + fac_us - ticks))
        {
            __NOP(); // 空操作，减少CPU消耗
        }
    }
    else
    {
        while ((start_tick - SysTick->VAL) < ticks)
        {
            __NOP(); // 空操作，减少CPU消耗
        }
    }
}

// void delay_ms(uint16_t nms)
// {
//     uint32_t ticks = 0;
//     uint32_t told = 0;
//     uint32_t tnow = 0;
//     uint32_t tcnt = 0;
//     uint32_t reload = 0;
//     reload = SysTick->LOAD;
//     ticks = nms * fac_ms;
//     told = SysTick->VAL;
//     while (1)
//     {
//         tnow = SysTick->VAL;
//         if (tnow != told)
//         {
//             if (tnow < told)
//             {
//                 tcnt += told - tnow;
//             }
//             else
//             {
//                 tcnt += reload - tnow + told;
//             }
//             told = tnow;
//             if (tcnt >= ticks)
//             {
//                 break;
//             }
//         }
//     }
// }
void delay_ms(uint16_t nms)
{
    osDelay(nms);
}