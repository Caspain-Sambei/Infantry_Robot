//
// Created by 18796 on 2026/2/28.
//

#ifndef GIMBAL_KALMANFILTER_H
#define GIMBAL_KALMANFILTER_H

#include <stdint.h>

// EKF 状态结构体 (7维状态: 四元数 q0~q3 + 陀螺零偏 bx,by,bz)
typedef struct {
    float q[4];         // 姿态四元数
    float bias[3];      // 陀螺仪零偏估计 (rad/s)
    float P[7*7];       // 协方差矩阵 (一维数组模拟 7x7)
    float dt;           // 时间步长 (s)
    uint8_t initialized;
} EKF_IMU;

// 初始化 EKF，传入初始四元数(通常为{1,0,0,0})和过程噪声/观测噪声配置
void EKF_IMU_Init(EKF_IMU *ekf, float init_q[4]);

// 核心更新函数，每收到一组 IMU 数据调用一次
// gyro: 三轴角速度 (rad/s)
// acc:  三轴加速度 (单位 g 或 m/s²，仅需方向一致)
// dt:   距上次调用时间间隔 (s)
void EKF_IMU_Update(EKF_IMU *ekf, float gyro[3], float acc[3], float dt);

// 从四元数计算欧拉角 (弧度)
void EKF_IMU_GetEuler(EKF_IMU *ekf, float *roll, float *pitch, float *yaw);

#endif //GIMBAL_KALMANFILTER_H
