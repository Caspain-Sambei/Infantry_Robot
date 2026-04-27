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
}Kalman_Filter_Structure;

// void Kalman_Filter(float *curr_data, float k);
extern void Kalman_Filter_Init(Kalman_Filter_Structure *kf, float init_value, float init_error, float R, float Q);
extern float Kalman_Filter(Kalman_Filter_Structure *reg, float current_measure);

float kalman_filter_1D(float Z_Measure, float Q_Input, float R_Input, float x0, float p0);
void Pass_Filter(float *curr_data, float k);

#endif //GIMBAL_FILTER_H