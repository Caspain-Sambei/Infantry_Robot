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
  volatile float gimbal_yaw = 0.0f, gimbal_pitch = 0.0f;
  volatile float dt7_x = 0.0f, dt7_y = 0.0f;
  float test_num = 0.01f; // 水平转动先给个相对小值

  /************************************************************
   *                   云台逆解算
   ************************************************************/
  if (rc_Data->rc.ch0 >= 1024 && rc_Data->rc.ch0 <= 1684)
  {
    gimbal_yaw = ((float)rc_Data->rc.ch0 - 1024.0f) / 660 * RC_TO_3508_Current * test_num;
  } 
  else if(rc_Data->rc.ch0 >= 364 && rc_Data->rc.ch0 < 1024)
  {
    gimbal_yaw = -((1024.0f - (float)rc_Data->rc.ch0) / 660) * RC_TO_3508_Current * test_num;
  }

  if (rc_Data->rc.ch1 >= 1024 && rc_Data->rc.ch1 <= 1684)
  {
    gimbal_pitch = ((float)rc_Data->rc.ch1 - 1024.0f) / 660 * RC_TO_3508_Current * test_num;
  } 
  else if(rc_Data->rc.ch1 >= 364 && rc_Data->rc.ch1 < 1024)
  {
    gimbal_pitch = -((1024.0f - (float)rc_Data->rc.ch1) / 660) * RC_TO_3508_Current * test_num;
  }

  /************************************************************
   *                   底盘逆解算
   ************************************************************/
  // 拨杆的绝对坐标
  if (rc_Data->rc.ch2 >= 1024 && rc_Data->rc.ch2 <= 1684) 
  {
    dt7_x = (float)((rc_Data->rc.ch2) - 1024) / 660.0f;
  }
  else if(rc_Data->rc.ch2 >= 364 && rc_Data->rc.ch2 < 1024)
  {
    dt7_x = -(float)(1024 - rc_Data->rc.ch2) / 660.0f;
  }

  if (rc_Data->rc.ch3 >= 1024 && rc_Data->rc.ch3 <= 1684) 
  {
    dt7_y = (float)(rc_Data->rc.ch3 - 1024) / 660.0f;
  }
  else if (rc_Data->rc.ch3 >= 364 && rc_Data->rc.ch3 < 1024)
  {
    dt7_y = -((float)(1024 - (rc_Data->rc.ch3)) / 660.0f);
  }
  // 绝对坐标系的线速度，单位 m/s
  chassis->inverse_speed_X = dt7_x * CHASSIS_MAX_SPEED;
  chassis->inverse_speed_Y = dt7_y * CHASSIS_MAX_SPEED;

  /************************************************************
   *              底盘正解算，判断角度和速度（世界坐标系下）
   ************************************************************/
  // 角度部分已经在gimbal.c的Startbmi088Task任务中实现;chassis->yaw_pid.outer.Actual

  // 底盘世界坐标系速度逆解算
  // 除以轮子到底盘中心的距离是计算单个电机对底盘旋转的角速度贡献 (rad/s)
  float Motor_1_Speed = (float)chassis->Motor_1_RxData.data2 * (2 * PI / 60.0f)
                        * WHEEL_RADIUS;
  float Motor_2_Speed = (float)chassis->Motor_2_RxData.data2 * (2 * PI / 60.0f)
                        * WHEEL_RADIUS;
  float Motor_3_Speed = (float)chassis->Motor_3_RxData.data2 * (2 * PI / 60.0f)
                        * WHEEL_RADIUS;
  float Motor_4_Speed = (float)chassis->Motor_4_RxData.data2 * (2 * PI / 60.0f)
                        * WHEEL_RADIUS;

  chassis->forward_speed_X =
      sqrtf(2.0f) / 4.0f *
      ((Motor_1_Speed - Motor_3_Speed) * (cosf(chassis->yaw_pid.outer.Actual) +
                                          sinf(chassis->yaw_pid.outer.Actual)) +
       (Motor_2_Speed - Motor_4_Speed) * (cosf(chassis->yaw_pid.outer.Actual) -
                                          sinf(chassis->yaw_pid.outer.Actual)));
  chassis->forward_speed_Y =
      sqrtf(2.0f) / 4.0f *
      ((Motor_1_Speed - Motor_3_Speed) * (sinf(chassis->yaw_pid.outer.Actual) -
                                          cosf(chassis->yaw_pid.outer.Actual)) +
       (Motor_2_Speed - Motor_4_Speed) * (cosf(chassis->yaw_pid.outer.Actual) +
                                          sinf(chassis->yaw_pid.outer.Actual)));
  chassis->forward_speed_X = Kalman_update(&chassis->Speed_X_KF, chassis->forward_speed_X);
  chassis->forward_speed_Y = Kalman_update(&chassis->Speed_Y_KF, chassis->forward_speed_Y);

  /************************************************************
   *              【核心修改1】航向角位置环控制
   ************************************************************/
  // 1. 计算拨杆的目标角度（绝对航向角，单位 rad）
  chassis->yaw_pid.outer.Target = atan2f(dt7_y, dt7_x);

  // 2. 航向角PID死区：如果摇杆归中，不进行航向控制，避免乱转
  float stick_magnitude = sqrtf(dt7_x * dt7_x + dt7_y * dt7_y);
  if (stick_magnitude < 0.1f)
  {
    // 摇杆归中，保持当前航向
    chassis->yaw_pid.outer.Target = chassis->yaw_pid.outer.Actual;
    chassis->target_omega = 0.0f;
  }
  else
  {
    PID_Update(&chassis->yaw_pid.outer, CHASSIS_MODE);
    // 4. PID输出就是目标自转角速度 (rad/s)
    chassis->target_omega = chassis->yaw_pid.outer.Out;
  }
  // 3. 平动速度环PID（X/Y方向分别闭环）
  chassis->Speed_X_PID.outer.Target = chassis->inverse_speed_X;
  chassis->Speed_X_PID.outer.Actual = chassis->forward_speed_X;
  PID_Update(&chassis->Speed_X_PID.outer, CHASSIS_MODE);

  chassis->Speed_Y_PID.outer.Target = chassis->inverse_speed_Y;
  chassis->Speed_Y_PID.outer.Actual = chassis->forward_speed_Y;
  PID_Update(&chassis->Speed_Y_PID.outer, CHASSIS_MODE);

  /************************************************************
   *                  底盘速度解算为通道值
   ************************************************************/
  float sin_ang = sinf(chassis->yaw_pid.outer.Actual);
  float cos_ang = cosf(chassis->yaw_pid.outer.Actual);

  speed_cal[0] =
    ((-cos_ang - sin_ang) * chassis->Speed_X_PID.outer.Out + (-sin_ang + cos_ang) * chassis->Speed_Y_PID.outer.Out
      + CHASSIS_RADIUS * chassis->target_omega) / sqrtf(2);
  speed_cal[1] =
    ((-cos_ang + sin_ang) * chassis->Speed_X_PID.outer.Out + (-sin_ang - cos_ang) * chassis->Speed_Y_PID.outer.Out
      + CHASSIS_RADIUS * chassis->target_omega) / sqrtf(2);
  speed_cal[2] =
    ((cos_ang + sin_ang) * chassis->Speed_X_PID.outer.Out + (sin_ang - cos_ang) * chassis->Speed_Y_PID.outer.Out
      + CHASSIS_RADIUS * chassis->target_omega) / sqrtf(2);
  speed_cal[3] =
    ((cos_ang - sin_ang) * chassis->Speed_X_PID.outer.Out + (sin_ang + cos_ang) * chassis->Speed_Y_PID.outer.Out
      + CHASSIS_RADIUS * chassis->target_omega) / sqrtf(2);
  // 通道值限幅
  for(uint8_t i = 0 ;i < 4;i++)
  {
    speed_cal[i] = speed_cal[i] / (2 * PI / 60.0f) / WHEEL_RADIUS;
    if(speed_cal[i] > OMNI_3508_CAL_MAX)
    {
      speed_cal[i] = OMNI_3508_CAL_MAX;
    }
    if(speed_cal[i] < -OMNI_3508_CAL_MAX)
    {
      speed_cal[i] = -OMNI_3508_CAL_MAX;
    }
  }
}

void Chassis_AutoRotate()
{
  if (p_reg->rc_Data.rc.s2 == 2)
  {

  }
}