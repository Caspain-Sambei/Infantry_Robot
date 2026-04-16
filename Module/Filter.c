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
// void Low_Pass_Filter(float *curr_data, float k)
// {
//     static float last_data = 0.0f;
//     static uint8_t first_call = 1;
//
//     if (first_call)
//     {
//         last_data = *curr_data;
//         first_call = 0;
//     }
//     *curr_data = k * (*curr_data) + (1.0f - k) * last_data;
//     last_data = *curr_data;
// }

/**
 * @brief 初始化卡尔曼滤波器参数
 *
 * @param kf 卡尔曼滤波器结构体指针
 * @param init_value 初始估计值，通常为第一次测量值或期望的初始状态
 * @param init_error 初始估计误差协方差，反映对初始值的置信程度
 *                   值越大表示对初始值越不信任，滤波器收敛越快但初期波动较大
 * @param R 测量噪声协方差（测量方差），反映传感器测量的可信度
 *          值越大表示测量噪声越大，滤波器更依赖预测值
 * @param Q 过程噪声协方差（过程方差），反映系统模型的不确定性
 *          值越大表示系统动态变化越快，滤波器更依赖测量值
 *
 * @note 该函数应在首次使用卡尔曼滤波器前调用一次
 * @note 参数调优建议：
 *       - R/Q 比值决定滤波器的平滑程度和响应速度
 *       - R 较大时滤波效果好但响应慢，Q 较大时响应快但噪声抑制差
 */
void Low_Pass_Filter_Init(Low_Pass_Filter_Structure *kf, float init_value, float init_error, float R, float Q)
{
    kf->last_aveData = init_value;
    kf->est_error = init_error;
    kf->pre_error = R;
    kf->Q = Q;
    kf->k = 0.0f;
}

float Low_Pass_Filter(Low_Pass_Filter_Structure *reg, float current_measure)
{
    // 0,更新测量值
    reg->pre_menData = current_measure;
    // 1，更新增益K
    reg->k = reg->est_error / (reg->est_error + reg->pre_error);
    // 2,计算本轮平均值
    reg->pre_aveData = reg->last_aveData + reg->k *(reg->pre_menData - reg->last_aveData);
    // 3，更新预估误差
    reg->est_error = (1 - reg->k) * reg->est_error + reg->Q;

    reg->last_aveData = reg->pre_aveData;
    return reg->pre_aveData;
}

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
void Pass_Filter(float *curr_data, float k)
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