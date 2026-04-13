//
// Created by 18796 on 2026/4/13.
//
#include "Filter.h"
#include <stdint.h>

/**
 * @brief 一阶低通滤波器
 *
 * 使用指数加权移动平均算法对输入数据进行滤波处理，
 * 滤除高频噪声，保留低频信号分量。
 *
 * @param curr_data 输入/输出数据指针，指向当前采样值，滤波后结果会更新到此位置
 * @param k 滤波系数，范围[0, 1]，值越大响应越快但滤波效果越弱，值越小滤波越强但延迟越大
 *
 * @note 函数内部使用静态变量维护历史状态，不支持多线程安全调用
 * @note 首次调用时会自动初始化历史数据为当前输入值
 */
void Low_Pass_Filter(float *curr_data, float k)
{
    static float last_data = 0.0f;
    static uint8_t first_call = 1;

    if (first_call)
    {
        last_data = *curr_data;
        first_call = 0;
    }
    *curr_data = k * (*curr_data) + (1.0f - k) * last_data;
    last_data = *curr_data;
}
