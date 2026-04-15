//
// Created by 18796 on 2026/4/13.
//

#ifndef GIMBAL_FILTER_H
#define GIMBAL_FILTER_H

typedef struct
{
    float pre_menData;
    float pre_aveData, last_aveData;
    float pre_error,est_error;          //前者：测量噪声方差
    float k,Q;                          //增益，过程噪声方差
}Low_Pass_Filter_Structure;

// void Low_Pass_Filter(float *curr_data, float k);
extern void Low_Pass_Filter_Init(Low_Pass_Filter_Structure *kf, float init_value, float init_error, float R, float Q);
extern float Low_Pass_Filter(Low_Pass_Filter_Structure *reg, float current_measure);

#endif //GIMBAL_FILTER_H