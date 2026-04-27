//
// Created by 18796 on 2026/4/15.
//

#ifndef GIMBAL_INS_H
#define GIMBAL_INS_H

void INS_QuaternionToEuler(const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif //GIMBAL_INS_H
