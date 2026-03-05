
//
// Created by 18796 on 2026/2/28.
//
#include <stdint.h>
#include "KalmanFilter.h"

// 初始化卡尔曼滤波器
void Kalman_Init(Kalman_Filter *kf, float init_value, float init_error, float R, float Q)
{
    kf->last_aveData = init_value;
    kf->est_error = init_error;
    kf->pre_error = R;
    kf->Q = Q;
    kf->k = 0.0f;
}

float Kalman_update(Kalman_Filter *reg)
{
    // 0,更新测量值
    //reg->pre_menData = ;
    // 1，更新增益K
    reg->k = reg->est_error / (reg->est_error + reg->pre_error);
    // 2,计算本轮平均值
    reg->pre_aveData = reg->last_aveData
                + reg->k *(reg->pre_menData - reg->last_aveData);
    // 3，更新预估误差
    reg->est_error = (1 - reg->k) * reg->est_error;

    reg->last_aveData = reg->pre_aveData;
    return reg->pre_aveData;
}