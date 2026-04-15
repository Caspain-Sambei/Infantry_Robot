//
// Created by 18796 on 2026/2/28.
//
#include <stdint.h>
#include "KalmanFilter.h"

#include "KalmanFilter.h"
#include <math.h>
#include <string.h>

// 噪声参数 (需根据实际传感器调试)
#define Q_W     (0.3f)      // 过程噪声：角速度噪声 (rad/s)
#define Q_B     (0.001f)    // 过程噪声：零偏漂移噪声 (rad/s²)
#define R_ACC   (0.5f)      // 观测噪声：加速度计噪声 (g)

// 矩阵运算辅助宏 (简化写法)
#define IDX(i,j) ((i)*7 + (j))

// 向量叉乘
static inline void cross(float a[3], float b[3], float c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

// 四元数乘法: out = a * b
static void quat_mul(float a[4], float b[4], float out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// 四元数归一化
static void quat_normalize(float q[4])
{
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 1e-6f)
    {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

void EKF_IMU_Init(EKF_IMU* ekf, float init_q[4])
{
    memcpy(ekf->q, init_q, 4 * sizeof(float));
    memset(ekf->bias, 0, 3 * sizeof(float));
    memset(ekf->P, 0, 49 * sizeof(float));
    // 初始化协方差矩阵 P 为对角阵
    for (int i = 0; i < 7; i++)
    {
        ekf->P[IDX(i, i)] = (i < 4) ? 0.1f : 0.01f; // 姿态不确定度0.1，零偏不确定度0.01
    }
    ekf->initialized = 1;
}

// 状态转移函数：根据角速度预测下一时刻四元数
static void predict_state(float q[4], float gyro[3], float bias[3], float dt)
{
    float w[3] = {gyro[0] - bias[0], gyro[1] - bias[1], gyro[2] - bias[2]};
    float norm_w = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (norm_w > 1e-6f)
    {
        float angle = norm_w * dt;
        float ca = cosf(angle / 2);
        float sa = sinf(angle / 2);
        float dq[4] = {ca, sa * w[0] / norm_w, sa * w[1] / norm_w, sa * w[2] / norm_w};
        float new_q[4];
        quat_mul(q, dq, new_q);
        memcpy(q, new_q, 4 * sizeof(float));
    }
    quat_normalize(q);
}

// 计算雅可比矩阵 F (7x7) 和过程噪声协方差 Qd (7x7)
static void compute_F_and_Q(float q[4], float gyro[3], float bias[3], float dt, float F[49], float Qd[49])
{
    float w[3] = {gyro[0]-bias[0], gyro[1]-bias[1], gyro[2]-bias[2]};
    // 简化的F矩阵：仅考虑四元数动力学与零偏相关项
    memset(F, 0, 49*sizeof(float));
    // F 左上角 4x4 是四元数对自身的偏导
    F[IDX(0,0)] = 1;                F[IDX(0,1)] = -0.5f*dt*w[0];  F[IDX(0,2)] = -0.5f*dt*w[1];  F[IDX(0,3)] = -0.5f*dt*w[2];
    F[IDX(1,0)] = 0.5f*dt*w[0];     F[IDX(1,1)] = 1;              F[IDX(1,2)] = 0.5f*dt*w[2];   F[IDX(1,3)] = -0.5f*dt*w[1];
    F[IDX(2,0)] = 0.5f*dt*w[1];     F[IDX(2,1)] = -0.5f*dt*w[2];  F[IDX(2,2)] = 1;              F[IDX(2,3)] = 0.5f*dt*w[0];
    F[IDX(3,0)] = 0.5f*dt*w[2];     F[IDX(3,1)] = 0.5f*dt*w[1];   F[IDX(3,2)] = -0.5f* dt * w[0];
    F[IDX(3, 3)] = 1;
    // F 右上角 4x3 是四元数对零偏的偏导
    for (int i = 0; i < 4; i++)
    {
        F[IDX(i, 4)] = 0.5f * dt * q[i]; // 简化处理，更精确的需要计算四元数导数对零偏的雅可比
    }
    // 零偏部分：单位阵
    for (int i = 4; i < 7; i++) F[IDX(i, i)] = 1;

    // Qd 离散化过程噪声
    float G[7 * 3] = {0}; // 噪声驱动矩阵
    // 四元数部分受角速度噪声影响
    G[IDX(0, 0)] = -0.5f * dt * q[1];   G[IDX(0, 1)] = -0.5f * dt * q[2];   G[IDX(0, 2)] = -0.5f * dt * q[3];
    G[IDX(1, 0)] = 0.5f * dt * q[0];    G[IDX(1, 1)] = -0.5f * dt * q[3];   G[IDX(1, 2)] = 0.5f * dt * q[2];
    G[IDX(2, 0)] = 0.5f * dt * q[3];    G[IDX(2, 1)] = 0.5f * dt * q[0];    G[IDX(2, 2)] = -0.5f * dt * q[1];
    G[IDX(3, 0)] = -0.5f * dt * q[2];   G[IDX(3, 1)] = 0.5f * dt * q[1];    G[IDX(3, 2)] = 0.5f * dt * q[0];
    // 零偏部分噪声
    G[IDX(4, 0)] = dt;                  G[IDX(5, 1)] = dt;                  G[IDX(6, 2)] = dt;

    float Qc[9] = {Q_W * Q_W, 0, 0, 0,Q_W * Q_W, 0, 0, 0,Q_B * Q_B};
    // Qd = G * Qc * G^T
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            Qd[IDX(i, j)] = 0;
            for (int k = 0; k < 3; k++)
            {
                Qd[IDX(i, j)] += G[IDX(i, k)] * Qc[k * 3 + k] * G[IDX(j, k)];
            }
        }
    }
}

// 观测模型：预测重力方向
static void predict_measurement(float q[4], float z_pred[3])
{
    // 将参考重力向量 [0,0,1] 旋转到机体坐标系
    float g_ref[3] = {0, 0, 1};
    float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
    float q_g[4] = {0, g_ref[0], g_ref[1], g_ref[2]};
    float tmp[4], res[4];
    quat_mul(q, q_g, tmp);
    quat_mul(tmp, q_conj, res);
    z_pred[0] = res[1];
    z_pred[1] = res[2];
    z_pred[2] = res[3];
}

// 计算观测雅可比 H (3x7)
static void compute_H(float q[4], float H[21])
{
    // 简化的 H 矩阵，仅对四元数部分求偏导
    memset(H, 0, 21 * sizeof(float));
    H[0 * 7 + 0] = 2 * q[2];
    H[0 * 7 + 1] = -2 * q[3];
    H[0 * 7 + 2] = 2 * q[0];
    H[0 * 7 + 3] = -2 * q[1];
    H[1 * 7 + 0] = -2 * q[1];
    H[1 * 7 + 1] = -2 * q[0];
    H[1 * 7 + 2] = -2 * q[3];
    H[1 * 7 + 3] = -2 * q[2];
    H[2 * 7 + 0] = -2 * q[0];
    H[2 * 7 + 1] = 2 * q[1];
    H[2 * 7 + 2] = 2 * q[2];
    H[2 * 7 + 3] = -2 * q[3];
}

void EKF_IMU_Update(EKF_IMU* ekf, float gyro[3], float acc[3], float dt)
{
    if (!ekf->initialized) return;
    ekf->dt = dt;

    // ========== 1. 预测步骤 ==========
    float q_prior[4];
    memcpy(q_prior, ekf->q, 4 * sizeof(float));
    predict_state(ekf->q, gyro, ekf->bias, dt);

    float F[49], Qd[49];
    compute_F_and_Q(q_prior, gyro, ekf->bias, dt, F, Qd);

    // P = F * P * F' + Qd
    float P_new[49] = {0};
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            float sum = 0;
            for (int k = 0; k < 7; k++)
            {
                sum += F[IDX(i, k)] * ekf->P[IDX(k, j)];
            }
            P_new[IDX(i, j)] = sum;
        }
    }
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            float sum = 0;
            for (int k = 0; k < 7; k++)
            {
                sum += P_new[IDX(i, k)] * F[IDX(j, k)]; // F' 转置
            }
            ekf->P[IDX(i, j)] = sum + Qd[IDX(i, j)];
        }
    }

    // ========== 2. 更新步骤（仅当加速度有效） ==========
    float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (acc_norm > 0.1f && acc_norm < 2.0f)
    {
        // 避免加速度过大或过小
        float acc_n[3] = {acc[0] / acc_norm, acc[1] / acc_norm, acc[2] / acc_norm};

        // 预测测量值
        float z_pred[3];
        predict_measurement(ekf->q, z_pred);

        // 计算新息
        float y[3] = {acc_n[0] - z_pred[0], acc_n[1] - z_pred[1], acc_n[2] - z_pred[2]};

        // 计算 H 矩阵
        float H[21];
        compute_H(ekf->q, H);

        // 计算 S = H * P * H' + R
        float S[9] = {0};
        float R_diag[3] = {R_ACC, R_ACC, R_ACC};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0;
                for (int k = 0; k < 7; k++)
                {
                    for (int l = 0; l < 7; l++)
                    {
                        sum += H[i * 7 + k] * ekf->P[IDX(k, l)] * H[j * 7 + l];
                    }
                }
                S[i * 3 + j] = sum + ((i == j) ? R_diag[i] : 0);
            }
        }

        // 计算卡尔曼增益 K = P * H' * inv(S)
        float K[21] = {0};
        // 先计算 P * H'
        float PHt[7 * 3] = {0};
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0;
                for (int k = 0; k < 7; k++)
                {
                    sum += ekf->P[IDX(i, k)] * H[j * 7 + k];
                }
                PHt[i * 3 + j] = sum;
            }
        }
        // 简易 3x3 矩阵求逆（S 对称正定）
        float det = S[0] * S[4] * S[8] + S[1] * S[5] * S[6] + S[2] * S[3] * S[7]
            - S[2] * S[4] * S[6] - S[1] * S[3] * S[8] - S[0] * S[5] * S[7];
        if (fabsf(det) > 1e-6f)
        {
            float invS[9];
            invS[0] = (S[4] * S[8] - S[5] * S[7]) / det;
            invS[1] = (S[2] * S[7] - S[1] * S[8]) / det;
            invS[2] = (S[1] * S[5] - S[2] * S[4]) / det;
            invS[3] = (S[5] * S[6] - S[3] * S[8]) / det;
            invS[4] = (S[0] * S[8] - S[2] * S[6]) / det;
            invS[5] = (S[2] * S[3] - S[0] * S[5]) / det;
            invS[6] = (S[3] * S[7] - S[4] * S[6]) / det;
            invS[7] = (S[1] * S[6] - S[0] * S[7]) / det;
            invS[8] = (S[0] * S[4] - S[1] * S[3]) / det;

            for (int i = 0; i < 7; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    float sum = 0;
                    for (int k = 0; k < 3; k++)
                    {
                        sum += PHt[i * 3 + k] * invS[k * 3 + j];
                    }
                    K[i * 3 + j] = sum;
                }
            }
        }

        // 状态更新: dx = K * y
        float dx[7] = {0};
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                dx[i] += K[i * 3 + j] * y[j];
            }
        }

        // 应用更新到四元数和零偏
        ekf->q[0] += dx[0];
        ekf->q[1] += dx[1];
        ekf->q[2] += dx[2];
        ekf->q[3] += dx[3];
        quat_normalize(ekf->q);
        ekf->bias[0] += dx[4];
        ekf->bias[1] += dx[5];
        ekf->bias[2] += dx[6];

        // 协方差更新: P = (I - K*H) * P
        float I_KH[49];
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 7; j++)
            {
                I_KH[IDX(i, j)] = (i == j) ? 1.0f : 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    I_KH[IDX(i, j)] -= K[i * 3 + k] * H[k * 7 + j];
                }
            }
        }
        float P_tmp[49];
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 7; j++)
            {
                float sum = 0;
                for (int k = 0; k < 7; k++)
                {
                    sum += I_KH[IDX(i, k)] * ekf->P[IDX(k, j)];
                }
                P_tmp[IDX(i, j)] = sum;
            }
        }
        memcpy(ekf->P, P_tmp, 49 * sizeof(float));
    }
}

void EKF_IMU_GetEuler(EKF_IMU* ekf, float* roll, float* pitch, float* yaw)
{
    float q0 = ekf->q[0], q1 = ekf->q[1], q2 = ekf->q[2], q3 = ekf->q[3];
    *roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    *pitch = asinf(2 * (q0 * q2 - q3 * q1));
    *yaw = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}
