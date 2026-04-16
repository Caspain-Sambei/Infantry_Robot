//
// Created by 18796 on 2026/3/28.
//
#include "Omni_wheel.h"
#include "reg.h"
#include "KalmanFilter.h"
/**
 * @brief 全向轮底盘/云台解算，23通道实现水平平动，01通道对应云台yaw,pitch
 * @param rc_Data 结构体由dr16手册提供
 * @param chassis 详见reg.h
 * @param speed_cal 发送给电机的电流值
 * @note 全向轮;默认装配方向：一侧同向，两侧镜像
 * @note 逆时针为正；1~4：左上，右上，左下，右下
 */
void Omni_wheel_calculate(const RC_Ctl_t *rc_Data,CHASSIS *chassis,float speed_cal[4])
{

  float dt7_x = 0.0f, dt7_y = 0.0f;
  float rotate_x= 0.0f, rotate_y = 0.0f;
  /************************************************************
   *                   小陀螺
   ************************************************************/
  if (rc_Data->rc.ch0 >= 364 && rc_Data->rc.ch0 <= 1684)
  {
    rotate_x = (float)((rc_Data->rc.ch0) - 1024) / 660.0f;
  }
  if (rc_Data->rc.ch1 >= 364 && rc_Data->rc.ch1 <= 1684)
  {
    rotate_y = (float)((rc_Data->rc.ch1) - 1024) / 660.0f;
  }

  /************************************************************
   *                   通道值归一化，映射角度
   ************************************************************/
  // 拨杆的绝对坐标
  if (rc_Data->rc.ch2 >= 364 && rc_Data->rc.ch2 <= 1684)
  {
    dt7_x = (float)((rc_Data->rc.ch2) - 1024) / 660.0f;
  }
  if (rc_Data->rc.ch3 >= 364 && rc_Data->rc.ch3 <= 1684)
  {
    dt7_y = (float)(rc_Data->rc.ch3 - 1024) / 660.0f;
  }

  /************************************************************
   *              获取目标角度,计算单环PID
   ************************************************************/
  // 绝对坐标系的线速度，单位 m/s
  chassis->inverse_speed_X = dt7_x * CHASSIS_MAX_SPEED;
  chassis->inverse_speed_Y = dt7_y * CHASSIS_MAX_SPEED;

  chassis->Speed_X_PID.outer.Target = chassis->inverse_speed_X;
  chassis->Speed_Y_PID.outer.Target = chassis->inverse_speed_Y;

  // chassis->yaw_pid.inner.Target = atan2f(dt7_x, dt7_y);
  chassis->yaw_pid.outer.Target = atan2f(rotate_x, rotate_y) * 57.32f;
  chassis->yaw_pid.outer.Target =-chassis->yaw_pid.outer.Target;

  //chassis->yaw_pid.inner.Actual已经在bmi088RTOS中赋值

  /************************************************************
   *              底盘正解算，判断角度和速度，自身坐标系！！！
   ************************************************************/
  // 除以轮子到底盘中心的距离是计算单个电机对底盘旋转的角速度贡献 (rad/s)
  float Motor_1_Speed =
    (float)chassis->Motor_1_RxData.data2 / REDUCTION_RATIO_3508
    * (2 * PI / 60.0f) * WHEEL_RADIUS;
  float Motor_2_Speed =
    (float)chassis->Motor_2_RxData.data2 / REDUCTION_RATIO_3508
    * (2 * PI / 60.0f) * WHEEL_RADIUS;
  float Motor_3_Speed =
    (float)chassis->Motor_3_RxData.data2 / REDUCTION_RATIO_3508
    * (2 * PI / 60.0f) * WHEEL_RADIUS;
  float Motor_4_Speed =
    (float)chassis->Motor_4_RxData.data2 / REDUCTION_RATIO_3508
    * (2 * PI / 60.0f) * WHEEL_RADIUS;

  /************************************************************
   *                  世界坐标系
   ************************************************************/
  // chassis->forward_speed_X =
  //     sqrtf(2.0f) / 4.0f *
  //     ((Motor_1_Speed - Motor_3_Speed) * (cosf(chassis->yaw_pid.inner.Actual) +
  //                                         sinf(chassis->yaw_pid.inner.Actual)) -
  //      (Motor_2_Speed - Motor_4_Speed) * (cosf(chassis->yaw_pid.inner.Actual) -
  //                                         sinf(chassis->yaw_pid.inner.Actual)));
  // chassis->forward_speed_Y =
  //     sqrtf(2.0f) / 4.0f *
  //     ((Motor_1_Speed - Motor_3_Speed) * (sinf(chassis->yaw_pid.inner.Actual) -
  //                                         cosf(chassis->yaw_pid.inner.Actual)) +
  //      (Motor_2_Speed - Motor_4_Speed) * (cosf(chassis->yaw_pid.inner.Actual) +
  //                                         sinf(chassis->yaw_pid.inner.Actual)));

  /************************************************************
   *                  底盘自身坐标系在的速度和目标角度
   ************************************************************/
  chassis->forward_speed_X =
    sqrtf(2.0f) / 4.0f * (Motor_1_Speed - Motor_3_Speed - Motor_2_Speed + Motor_4_Speed);
  chassis->forward_speed_Y =
    sqrtf(2.0f) / 4.0f * (Motor_1_Speed - Motor_3_Speed + Motor_2_Speed - Motor_4_Speed);
  // 一阶低通滤波
  Pass_Filter(&chassis->forward_speed_X,0.5f);
  Pass_Filter(&chassis->forward_speed_Y,0.5f);

  chassis->Speed_X_PID.inner.Actual = chassis->forward_speed_X;
  chassis->Speed_Y_PID.inner.Actual = chassis->forward_speed_Y;

  PID_Update(&chassis->Speed_X_PID.inner, CHASSIS_MODE);
  PID_Update(&chassis->Speed_Y_PID.inner, CHASSIS_MODE);

  /************************************************************
   *                  限速保护
   ************************************************************/
  if (fabs(chassis->Speed_X_PID.inner.Actual) > LIMIT_SPEED || fabs(chassis->Speed_Y_PID.inner.Actual) > LIMIT_SPEED
    || fabs(Motor_1_Speed) > LIMIT_SPEED || fabs(Motor_2_Speed) > LIMIT_SPEED
    || fabs(Motor_3_Speed) > LIMIT_SPEED || fabs(Motor_4_Speed) > LIMIT_SPEED)
  {
    speed_cal[0] = 0.0f;  speed_cal[1] = 0.0f;  speed_cal[2] = 0.0f;  speed_cal[3] = 0.0f;
    return;
  }

  float stick_magnitude = sqrtf(rotate_x * rotate_x + rotate_y * rotate_y);
  if (stick_magnitude < 1.0f)
  {
    chassis->yaw_pid.inner.Target = chassis->yaw_pid.inner.Actual;
    // 摇杆归中时，仍需运行航向 PID 闭环，让底盘自动抵抗外界扰动，维持当前朝向
  }
  // PID_Update(&chassis->yaw_pid.inner, CHASSIS_MODE);
  // chassis->target_omega = chassis->yaw_pid.inner.Out;

  /************************************************************
   *                  底盘速度解算为通道值
   ************************************************************/
  float sin_ang = sinf(chassis->yaw_pid.inner.Actual);
  float cos_ang = cosf(chassis->yaw_pid.inner.Actual);
  // speed_cal[0] =
  //   ((-cos_ang - sin_ang) * chassis->Speed_X_PID.inner.Out + (-sin_ang + cos_ang) * chassis->Speed_Y_PID.inner.Out
  //     + CHASSIS_RADIUS * chassis->target_omega) * sqrtf(2) / 2;
  // speed_cal[1] =
  //   ((-cos_ang + sin_ang) * chassis->Speed_X_PID.inner.Out + (-sin_ang - cos_ang) * chassis->Speed_Y_PID.inner.Out
  //     + CHASSIS_RADIUS * chassis->target_omega) * sqrtf(2) / 2;
  // speed_cal[2] =
  //   ((cos_ang + sin_ang) * chassis->Speed_X_PID.inner.Out + (sin_ang - cos_ang) * chassis->Speed_Y_PID.inner.Out
  //     + CHASSIS_RADIUS * chassis->target_omega) * sqrtf(2) / 2;
  // speed_cal[3] =
  //   ((cos_ang - sin_ang) * chassis->Speed_X_PID.inner.Out + (sin_ang + cos_ang) * chassis->Speed_Y_PID.inner.Out
  //     + CHASSIS_RADIUS * chassis->target_omega) * sqrtf(2) / 2;
  speed_cal[0] =
  (chassis->Speed_X_PID.inner.Out + chassis->Speed_Y_PID.inner.Out
    + CHASSIS_DIAGONAL_DISTANCE * chassis->target_omega) * sqrtf(2) / 2;
  speed_cal[1] =
  (-chassis->Speed_X_PID.inner.Out + chassis->Speed_Y_PID.inner.Out
    + CHASSIS_DIAGONAL_DISTANCE * chassis->target_omega) * sqrtf(2) / 2;
  speed_cal[2] =
  (-chassis->Speed_X_PID.inner.Out - chassis->Speed_Y_PID.inner.Out
    + CHASSIS_DIAGONAL_DISTANCE * chassis->target_omega) * sqrtf(2) / 2;
  speed_cal[3] =
  (chassis->Speed_X_PID.inner.Out - chassis->Speed_Y_PID.inner.Out
    + CHASSIS_DIAGONAL_DISTANCE * chassis->target_omega) * sqrtf(2) / 2;

  /************************************************************
  *                  通道限幅
  ************************************************************/
  for(uint8_t i = 0 ;i < 4;i++)
  {
    speed_cal[i] = speed_cal[i] / (2 * PI / 60.0f) / WHEEL_RADIUS * REDUCTION_RATIO_3508;
    
    if(speed_cal[i] > OMNI_3508_CAL_MAX)
    {
      speed_cal[i] = OMNI_3508_CAL_MAX;
    }
    if(speed_cal[i] < -OMNI_3508_CAL_MAX)
    {
      speed_cal[i] = -OMNI_3508_CAL_MAX;
    }
  }
  /************************************************************
  *                  死区保护
  ************************************************************/
  // if (fabsf(chassis->Speed_X_PID.inner.Target) <= 0.01f || fabsf(chassis->Speed_Y_PID.inner.Target) <= 0.01f)
  // {
  //   for(uint8_t i = 0 ;i < 4;i++)
  //   {
  //     speed_cal[i] = 0.0f;
  //   }
  // }

}
