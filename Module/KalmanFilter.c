#include "KalmanFilter.h"
#include <math.h>
#include <string.h>

// ========== 噪声参数（需根据实际传感器调试） ==========
#define Q_GYRO   0.000005f     // 角速度过程噪声 (rad/s)
#define Q_BIAS   0.00095f   // 零偏漂移噪声 (rad/s²)
#define R_ACC    0.001f    // 加速度计观测噪声 (g²)

// 矩阵辅助宏（6x6 按行优先存储）
#define IDX(i,j) ((i)*6 + (j))

// ========== 基础数学函数 ==========
static inline void cross(const float a[3], const float b[3], float c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

static void quat_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 0.0f) {n = 0.0f;}
    n = sqrtf(n);
    if (n > 1e-6f) {
        q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
    }
}

// ========== 矩阵运算：C = A * B (A, B, C 均为 6×6) ==========
static void mat_mul_6x6(const float A[36], const float B[36], float C[36])
{
    memset(C, 0, 36*sizeof(float));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                C[IDX(i,j)] += A[IDX(i,k)] * B[IDX(k,j)];
}

// ========== 初始化 ==========
void EKF_IMU_Init(EKF_IMU* mekf, float init_q[4], float init_bias[3])
{
    memcpy(mekf->q, init_q, 4 * sizeof(float));
    memcpy(mekf->bias, init_bias, 3 * sizeof(float));
    memset(mekf->P, 0, 36 * sizeof(float));

    // P 初始值：姿态误差 0.1 rad²，零偏误差 0.01 (rad/s)²
    for (int i = 0; i < 3; i++) mekf->P[IDX(i,i)] = 0.1f;
    for (int i = 3; i < 6; i++) mekf->P[IDX(i,i)] = 0.01f;

    mekf->initialized = 1;
}

// ========== SO(3) 指数映射：从旋转向量到四元数增量 ==========
static void exp_so3(float delta[3], float dq[4])
{
    float n = sqrtf(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
    if (n < 0.0f) {n = 0.0f;}
    n = sqrtf(n);

    if (n < 1e-6f) {
        dq[0] = 1.0f; dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = 0.0f;
        return;
    }
    float hn = 0.5f * n;
    float s = sinf(hn) / n;
    dq[0] = cosf(hn);
    dq[1] = s * delta[0];
    dq[2] = s * delta[1];
    dq[3] = s * delta[2];
}

// ========== 主更新函数 ==========
void EKF_IMU_Update(EKF_IMU* mekf, float gyro[3], float acc[3], float dt)
{
    if (!mekf->initialized) return;
    mekf->dt = dt;

    // ------ 1. 名义状态预测 ------
    float w[3] = {
        gyro[0] - mekf->bias[0],
        gyro[1] - mekf->bias[1],
        gyro[2] - mekf->bias[2]
    };
    float wn = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    if (wn < 0.0f){wn = 0.0f;}
    wn = sqrtf(wn);

    float dq[4];
    if (wn > 1e-6f) {
        float ang = wn * dt;
        float s = sinf(0.5f * ang) / wn;
        dq[0] = cosf(0.5f * ang);
        dq[1] = s * w[0];
        dq[2] = s * w[1];
        dq[3] = s * w[2];
    } else {
        dq[0] = 1.0f; dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = 0.0f;
    }
    quat_mul(mekf->q, dq, mekf->q);
    quat_normalize(mekf->q);

    // ------ 2. 误差状态协方差预测 ------
    // 状态转移矩阵 F (6×6) 近似 I + A*dt
    // A 矩阵：上三角块为 -[w]× 和 -I，其余零
    float F[36];
    memset(F, 0, 36 * sizeof(float));
    for (int i = 0; i < 6; i++) F[IDX(i,i)] = 1.0f;

    // 上部 3×3：F01 = -[w]× * dt
    F[IDX(1,0)] =  w[2]*dt;   F[IDX(2,0)] = -w[1]*dt;
    F[IDX(0,1)] = -w[2]*dt;   F[IDX(2,1)] =  w[0]*dt;
    F[IDX(0,2)] =  w[1]*dt;   F[IDX(1,2)] = -w[0]*dt;

    // 上部右 3×3：-I * dt
    F[IDX(0,3)] = -dt;  F[IDX(1,4)] = -dt;  F[IDX(2,5)] = -dt;

    // 噪声驱动矩阵 G (6×6) 的离散化：这里简化为对角线块
    float G[36];
    memset(G, 0, 36 * sizeof(float));
    G[IDX(0,0)] = -dt;  G[IDX(1,1)] = -dt;  G[IDX(2,2)] = -dt;
    G[IDX(3,3)] =  dt;  G[IDX(4,4)] =  dt;  G[IDX(5,5)] =  dt;

    // 过程噪声协方差 Qc = diag(Q_GYRO²*I3, Q_BIAS²*I3)
    float Qc[36];
    memset(Qc, 0, 36 * sizeof(float));
    for (int i = 0; i < 3; i++) Qc[IDX(i,i)] = Q_GYRO * Q_GYRO;
    for (int i = 3; i < 6; i++) Qc[IDX(i,i)] = Q_BIAS * Q_BIAS;

    // Qd = G * Qc * G^T
    float Qd[36];
    memset(Qd, 0, 36 * sizeof(float));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                Qd[IDX(i,j)] += G[IDX(i,k)] * Qc[IDX(k,k)] * G[IDX(j,k)];

    // P = F * P * F^T + Qd
    float P_temp[36];
    mat_mul_6x6(F, mekf->P, P_temp);
    float FT[36];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            FT[IDX(i,j)] = F[IDX(j,i)];
    memset(mekf->P, 0, 36 * sizeof(float));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                mekf->P[IDX(i,j)] += P_temp[IDX(i,k)] * FT[IDX(k,j)];
    for (int i = 0; i < 36; i++) mekf->P[i] += Qd[i];

    // ------ 3. 观测更新（仅当加速度接近 1 g） ------
    float acc_norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (acc_norm < 0.0f) {acc_norm = 0.0f;}
    acc_norm = sqrtf(acc_norm);
    if (acc_norm > 0.85f && acc_norm < 1.15f) // ****************************************************************
    // if (acc_norm > 8.5f && acc_norm < 11.5f)
    {
        float acc_n[3] = {acc[0]/acc_norm, acc[1]/acc_norm, acc[2]/acc_norm};

        // 预测重力方向：将参考重力 [0,0,1] 旋转到机体坐标
        float g_ref[3] = {0, 0, 1};
        float q_conj[4] = {mekf->q[0], -mekf->q[1], -mekf->q[2], -mekf->q[3]};
        float q_g[4] = {0, g_ref[0], g_ref[1], g_ref[2]};
        float tmp[4], res[4];
        quat_mul(mekf->q, q_g, tmp);
        quat_mul(tmp, q_conj, res);
        float z_pred[3] = {res[1], res[2], res[3]};

        // 新息
        float y[3] = {
            acc_n[0] - z_pred[0],
            acc_n[1] - z_pred[1],
            acc_n[2] - z_pred[2]
        };

        // 观测雅可比 H：仅对姿态误差有偏导，对零偏为 0
        float H[18];   // 3×6
        memset(H, 0, 18 * sizeof(float));
        // H 对 δθ 的雅可比就是 [z_pred]×
        H[1] =  z_pred[2];  H[2]  = -z_pred[1];
        H[6] = -z_pred[2];  H[8]  =  z_pred[0];
        H[12]=  z_pred[1];  H[13] = -z_pred[0];

        // S = H * P * H^T + R
        float R[9] = {R_ACC, 0, 0, 0, R_ACC, 0, 0, 0, R_ACC};
        float S[9] = {0};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float sum = 0;
                for (int k = 0; k < 6; k++)
                    for (int l = 0; l < 6; l++)
                        sum += H[i*6+k] * mekf->P[IDX(k,l)] * H[j*6+l];
                S[i*3+j] = sum + R[i*3+j];
            }
        }

        // 计算卡尔曼增益 K = P * H^T * inv(S)  (K 为 6×3)
        float K[18] = {0};
        float PHt[18] = {0};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 6; k++)
                    PHt[i*3+j] += mekf->P[IDX(i,k)] * H[j*6+k];

        // 3×3 矩阵求逆
        float det = S[0]*S[4]*S[8] + S[1]*S[5]*S[6] + S[2]*S[3]*S[7]
                  - S[2]*S[4]*S[6] - S[1]*S[3]*S[8] - S[0]*S[5]*S[7];
        if (fabsf(det) > 1e-6f) {
            float invS[9];
            invS[0] = (S[4]*S[8]-S[5]*S[7])/det;  invS[1] = (S[2]*S[7]-S[1]*S[8])/det;
            invS[2] = (S[1]*S[5]-S[2]*S[4])/det;
            invS[3] = (S[5]*S[6]-S[3]*S[8])/det;  invS[4] = (S[0]*S[8]-S[2]*S[6])/det;
            invS[5] = (S[2]*S[3]-S[0]*S[5])/det;
            invS[6] = (S[3]*S[7]-S[4]*S[6])/det;  invS[7] = (S[1]*S[6]-S[0]*S[7])/det;
            invS[8] = (S[0]*S[4]-S[1]*S[3])/det;

            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                        K[i*3+j] += PHt[i*3+k] * invS[k*3+j];
        }

        // 误差状态更新：dx = K * y
        float dx[6];
        memset(dx, 0, 6*sizeof(float));
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                dx[i] += K[i*3+j] * y[j];

        // 注入观测 → 修正名义状态
        float dq_corr[4];
        exp_so3(dx, dq_corr);          // δθ → 四元数增量
        quat_mul(mekf->q, dq_corr, mekf->q);
        quat_normalize(mekf->q);

        mekf->bias[0] += dx[3];
        mekf->bias[1] += dx[4];
        mekf->bias[2] += dx[5];

        // 协方差更新：P = (I - K*H) * P
        float I_KH[36];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                I_KH[IDX(i,j)] = (i==j) ? 1.0f : 0.0f;
                for (int k = 0; k < 3; k++)
                    I_KH[IDX(i,j)] -= K[i*3+k] * H[k*6+j];
            }
        mat_mul_6x6(I_KH, mekf->P, P_temp);
        memcpy(mekf->P, P_temp, 36 * sizeof(float));
    }
}

// ========== 欧拉角输出 ==========
void EKF_IMU_GetEuler(const EKF_IMU* mekf, float* roll, float* pitch, float* yaw)
{
    float q0 = mekf->q[0], q1 = mekf->q[1], q2 = mekf->q[2], q3 = mekf->q[3];
    *roll  = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    float sinp = 2*(q0*q2 - q3*q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp);
    *yaw   = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}