//
// Created by 18796 on 2026/2/25.
//

#ifndef GIMBAL_GIMBAL_H
#define GIMBAL_GIMBAL_H
#include <stdint.h>

/***************************************************************
 *                     视觉通信
 ***************************************************************/
// typedef struct
// {
//     float yaw;
//     float pitch;
//     float roll;
// }SENDPACKET;

typedef enum {
    SENTRY_DISABLED = 0,
    SENTRY_ENABLED = 1,
} sentry_state;

void INS_QuaternionToEuler(const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg);
void PID_Switch(uint8_t mode,uint16_t switch_time);
void calibrate_gyro_bias(uint16_t samples,float *gyro_bias,float *accel_bias);

#endif //GIMBAL_GIMBAL_H