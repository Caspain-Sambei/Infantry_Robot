//
// Created by 18796 on 2026/4/17.
//

#ifndef GIMBAL_BSP_BMI088_H
#define GIMBAL_BSP_BMI088_H

#include <stdint.h>

uint8_t BMI088_SwitchToSPI(uint8_t *accel_id, uint8_t *gyro_id);
uint8_t BMI088_Gyro_ReadReg_Direct(uint8_t reg);
uint8_t BMI088_VerifyGyroInterrupt_Direct(void);

#endif //GIMBAL_BSP_BMI088_H
