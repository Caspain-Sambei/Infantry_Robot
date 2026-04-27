//
// Created by 18796 on 2026/4/15.
//
#include "INS.h"
#include <math.h>

/**
 * @brief 四元数转欧拉角：供上层模块直接读取更直观的 ROLL/PITCH/YAW。
 * @param q
 * @param roll_deg
 * @param pitch_deg
 * @param yaw_deg
 */
void INS_QuaternionToEuler(const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    float yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    float pitch;
    if (fabsf(sinp) >= 1.0f)
    {
        pitch = copysignf((float)M_PI / 2.0f, sinp);
    }
    else
    {
        pitch = asinf(sinp);
    }

    //弧度转角度
    *roll_deg = roll * 57.29578f;
    *pitch_deg = pitch * 57.29578f;
    *yaw_deg = yaw * 57.29578f;
}
