//
// Created by 18796 on 2026/4/13.
//
#include "Filter.h"

#include <stdbool.h>
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
// void Kalman_Filter(float *curr_data, float k)
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
void Kalman_Filter_Init(Kalman_Filter_Structure *kf, float init_value, float init_error, float R, float Q)
{
    kf->last_aveData = init_value;
    kf->est_error = init_error;
    kf->pre_error = R;
    kf->Q = Q;
    kf->k = 0.0f;
}


float Kalman_Filter(Kalman_Filter_Structure *reg, float current_measure)
{
    // 0,更新测量值 : 当前测量值,上一次滤波输出值
    reg->pre_menData = current_measure;
    // 1，更新增益K = 估计误差方差 / (估计误差方差 + 测量噪声方差)
    reg->k = reg->est_error / (reg->est_error + reg->pre_error);
    // 2,计算本轮平均值 = 估计值 + 增益 * (当前测量值 - 估计值)
    reg->pre_aveData = reg->last_aveData + reg->k *(reg->pre_menData - reg->last_aveData);
    // 3，更新预估误差 = (1 - 增益) * 预估误差
    reg->est_error = (1 - reg->k) * reg->est_error + reg->Q;

    reg->last_aveData = reg->pre_aveData;
    return reg->pre_aveData;
}

/**
 * @brief 一维卡尔曼滤波器
 * @note 每次调用都重新初始化，没起到滤波的作用
 * @param Z_Measure 测量值
 * @param Q_Input 预测过程协方差
 * @param R_Input 测量过程协方差
 * @param x0 初始状态最优值。一般取0
 * @param p0 初始状态最优值协方差值。一般取1，不可为0
 * @return 返回最优估计值
 */
float kalman_filter_1D(float Z_Measure, float Q_Input, float R_Input, float x0, float p0)
{
    float X_Predict;	//预测值
    float X_Optimal;	//最优值
    float P_Predict;	//预测值协方差矩阵
    float P_Optimal;	//最优质协方差矩阵
    float K;			//卡尔曼增益
    float Q;			//预测过程协方差
    float R;			//测量过程协方差
    bool IsInit = true;	//初始化标志位
    if(IsInit == true)	//初始化操作
    {
        IsInit = false;	//清除标志位
        X_Optimal = x0;	//初始化
        P_Optimal = p0;
        Q = Q_Input;
        R = R_Input;
    }
    //预测过程
    X_Predict = X_Optimal;		//预测值更新
    P_Predict = P_Optimal + Q;	//预测值协方差矩阵更新
	
    //更新过程
    K = P_Predict / (P_Predict + R);	//更新卡尔曼滤波增益	
    X_Optimal = X_Predict + K*(Z_Measure - X_Predict);	//更新最优估计值
    P_Optimal = (1-K)*P_Predict;		//更新最优估计值的协方差矩阵
    return X_Optimal;			//返回最优估计值
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