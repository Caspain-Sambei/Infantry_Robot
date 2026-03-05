//
// Created by 18796 on 2026/2/28.
//

#ifndef GIMBAL_KALMANFILTER_H
#define GIMBAL_KALMANFILTER_H


typedef struct
{
    float pre_menData;
    float pre_aveData, last_aveData;
    float pre_error,est_error;          //前者：测量噪声方差
    float k,Q;                          //增益，过程噪声方差
}Kalman_Filter;

extern float Kalman_update(Kalman_Filter *reg);

#endif //GIMBAL_KALMANFILTER_H
