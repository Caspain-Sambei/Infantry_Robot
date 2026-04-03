//
// Created by 18796 on 2026/3/28.
//
#include "Omni_wheel.h"

/**
 * @brief 全向轮底盘/云台解算，23通道实现水平平动，01通道对应云台yaw,pitch
 * @param rc_Data 结构体由dr16手册提供
 * @param chassis 详见reg.h
 * @param speed_cal 发送给电机的电流值
 */
void Omni_wheel_calculate(const RC_Ctl_t *rc_Data,CHASSIS *chassis,float speed_cal[4])
{
    /****************************************************************
     *             全向轮;默认装配方向：一侧同向，两侧镜像；逆时针为正
     *                      1~4：左上，右上，左下，右下
     ****************************************************************/
    float test_num = 0.1f;    // 水平转动先给个相对小值
    // 云台yaw和pitch
    float gimbal_yaw = (float)rc_Data->rc.ch0 * RC_TO_3508_Current * test_num;
    float gimbal_pitch = (float)rc_Data->rc.ch1 * RC_TO_3508_Current * test_num;
    /************************************************************
     *                   正解算
    ************************************************************/
    float dt7_x = ((float)(rc_Data->rc.ch2) - 1024.0f) / 660.0f;
    float dt7_y = ((float)(rc_Data->rc.ch3) - 1024.0f) / 660.0f;
    chassis->yaw_pid.outer.Target = atan2f(dt7_y, dt7_x);
    // 线速度单位 m/s
    chassis->speed_X = (float)rc_Data->rc.ch3 * RC_TO_3508_Current * test_num;
    chassis->speed_Y = (float)rc_Data->rc.ch2 * RC_TO_3508_Current * test_num;
    chassis->Speed_pid.inner.Target =
        sqrtf(chassis->speed_X * chassis->speed_X + chassis->speed_Y * chassis->speed_Y);
    /************************************************************
     *                  逆解算，判断角度和速度（世界坐标系下）
    ************************************************************/
    // 角度部分已经在gimbal.c的Startbmi088Task任务中实现;chassis->yaw_pid.outer.Actual

    // 底盘世界坐标系速度逆解算
    float Motor_1_Speed = (float)chassis->Motor_1_RxData.data2 * 2 * PI /60.0f / GEAR_RATIO_3508 * CHASSIS_RADIUS;
    float Motor_2_Speed = (float)chassis->Motor_2_RxData.data2 * 2 * PI /60.0f / GEAR_RATIO_3508 * CHASSIS_RADIUS;
    float Motor_3_Speed = (float)chassis->Motor_3_RxData.data2 * 2 * PI /60.0f / GEAR_RATIO_3508 * CHASSIS_RADIUS;
    float Motor_4_Speed = (float)chassis->Motor_4_RxData.data2 * 2 * PI /60.0f / GEAR_RATIO_3508 * CHASSIS_RADIUS;

    chassis->speed_X = -sqrtf(2.0f) / 4.0f
        * ((Motor_1_Speed - Motor_3_Speed)
        * (cosf(chassis->yaw_pid.outer.Actual) + sinf(chassis->yaw_pid.outer.Actual))
        +
        (Motor_2_Speed - Motor_4_Speed)
        * (cosf(chassis->yaw_pid.outer.Actual) - sinf(chassis->yaw_pid.outer.Actual)));
    chassis->speed_Y = -sqrtf(2.0f) / 4.0f
        * ((Motor_1_Speed - Motor_3_Speed)
        * (sinf(chassis->yaw_pid.outer.Actual) - cosf(chassis->yaw_pid.outer.Actual))
        +
        (Motor_2_Speed - Motor_4_Speed)
        * (cosf(chassis->yaw_pid.outer.Actual) + sinf(chassis->yaw_pid.outer.Actual)));
    // 获取底盘速度
    chassis->Speed_pid.inner.Actual =
        sqrtf(chassis->speed_X*chassis->speed_X + chassis->speed_Y*chassis->speed_Y);

    /************************************************************
     *                  单环PID
    ************************************************************/
    PID_Update(&chassis->Speed_pid.inner,chassis_mode);

    /************************************************************
     *                  底盘速度解算为通道值
    ************************************************************/
    //chassis->yaw_pid.outer.Actual = p_reg->gimbal.yaw_pid.outer.Actual;
    float sin_ang = sinf(chassis->yaw_pid.outer.Actual);
    float cos_ang = cosf(chassis->yaw_pid.outer.Actual);
    float speed_X = chassis->Speed_pid.inner.Out * cosf(chassis->yaw_pid.outer.Target);
    float speed_Y = chassis->Speed_pid.inner.Out * sinf(chassis->yaw_pid.outer.Target);

    speed_cal[0] = (float)((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y + CHASSIS_RADIUS  * chassis->target_omega)
                    / sqrtf(2)
                    / 2 / PI *60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
    speed_cal[1] = (float)((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y + CHASSIS_RADIUS  * chassis->target_omega)
                    / sqrtf(2)
                    / 2 / PI *60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
    speed_cal[2] = (float)((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y + CHASSIS_RADIUS  * chassis->target_omega)
                    / sqrtf(2)
                    / 2 / PI *60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
    speed_cal[3] = (float)((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + CHASSIS_RADIUS  * chassis->target_omega)
                    / sqrtf(2)
                    / 2 / PI *60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
}
