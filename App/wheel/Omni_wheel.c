//
// Created by 18796 on 2026/3/28.
//
#include "Omni_wheel.h"

/**
 * @brief 全向轮底盘/云台解算，23通道实现水平平动，01通道对应云台yaw,pitch
 * @param rc_Data 结构体由dr16手册提供
 * @param chassis 详见reg.h
 * @param speed_cal 发送给电机的电流值
 * @note 全向轮;默认装配方向：一侧同向，两侧镜像
 * @note 逆时针为正；1~4：左上，右上，左下，右下
 */
void Omni_wheel_calculate(RC_Ctl_t *rc_Data,CHASSIS *chassis,float speed_cal[4])
{
  volatile float gimbal_yaw = 0.0f, gimbal_pitch = 0.0f;
  volatile float dt7_x = 0.0f, dt7_y = 0.0f;
  float test_num = 1.0f; // 水平转动先给个相对小值
  /************************************************************
   *                   云台逆解算
   ************************************************************/
  if (rc_Data->rc.ch0 >= 1024 && rc_Data->rc.ch0 <= 1684)
  {
    gimbal_yaw = ((float)rc_Data->rc.ch0 - 1024.0f) / 660 * RC_TO_3508_Current * test_num;
  } 
  else if(rc_Data->rc.ch0 >= 364 && rc_Data->rc.ch0 < 1024)
  {
    gimbal_yaw = -(1024.0f - (float)rc_Data->rc.ch0) / 660* RC_TO_3508_Current * test_num;
  }
    

  if (rc_Data->rc.ch1 >= 1024 && rc_Data->rc.ch1 <= 1684)
  {
    gimbal_pitch = ((float)rc_Data->rc.ch1 - 1024.0f) / 660 * RC_TO_3508_Current * test_num;
  } 
  else if(rc_Data->rc.ch1 >= 364 && rc_Data->rc.ch1 < 1024)
  {
    gimbal_pitch = -(1024.0f - (float)rc_Data->rc.ch1) / 660 * RC_TO_3508_Current * test_num;
  }

  /************************************************************
   *                   底盘逆解算
   ************************************************************/
  if (rc_Data->rc.ch2 >= 1024 && rc_Data->rc.ch2 <= 1684) 
  {
    dt7_x = ((float)(rc_Data->rc.ch2) - 1024.0f) / 660.0f;
  }
  else if(rc_Data->rc.ch2 >= 364 && rc_Data->rc.ch2 < 1024)
  {
    dt7_x = -(1024.0f - (float)(rc_Data->rc.ch2)) / 660.0f; 
  }

  if (rc_Data->rc.ch3 >= 1024 && rc_Data->rc.ch3 <= 1684) 
  {
    dt7_y = ((float)(rc_Data->rc.ch3) - 1024.0f) / 660.0f;
  }
  else if(rc_Data->rc.ch3 >= 364 && rc_Data->rc.ch3 < 1024)
  {
    dt7_y = -(1024.0f - (float)(rc_Data->rc.ch3)) / 660.0f;
  }

  chassis->yaw_pid.outer.Target = atan2f(dt7_y, dt7_x);
  // 线速度单位 m/s
  chassis->inverse_speed_X = dt7_x * RC_TO_3508_Current * test_num;
  chassis->inverse_speed_Y = dt7_y * RC_TO_3508_Current * test_num;
  chassis->Speed_pid.inner.Target =
      sqrtf(chassis->inverse_speed_X * chassis->inverse_speed_X +
            chassis->inverse_speed_Y * chassis->inverse_speed_Y);
  /************************************************************
   *                  正解算，判断角度和速度（世界坐标系下）
   ************************************************************/
  // 角度部分已经在gimbal.c的Startbmi088Task任务中实现;chassis->yaw_pid.outer.Actual

  // 底盘世界坐标系速度逆解算
  float Motor_1_Speed = (float)chassis->Motor_1_RxData.data2 * 2 * PI / 60.0f /
                        GEAR_RATIO_3508 * CHASSIS_RADIUS;
  float Motor_2_Speed = (float)chassis->Motor_2_RxData.data2 * 2 * PI / 60.0f /
                        GEAR_RATIO_3508 * CHASSIS_RADIUS;
  float Motor_3_Speed = (float)chassis->Motor_3_RxData.data2 * 2 * PI / 60.0f /
                        GEAR_RATIO_3508 * CHASSIS_RADIUS;
  float Motor_4_Speed = (float)chassis->Motor_4_RxData.data2 * 2 * PI / 60.0f /
                        GEAR_RATIO_3508 * CHASSIS_RADIUS;

  chassis->forward_speed_X =
      -sqrtf(2.0f) / 4.0f *
      ((Motor_1_Speed - Motor_3_Speed) * (cosf(chassis->yaw_pid.outer.Actual) +
                                          sinf(chassis->yaw_pid.outer.Actual)) +
       (Motor_2_Speed - Motor_4_Speed) * (cosf(chassis->yaw_pid.outer.Actual) -
                                          sinf(chassis->yaw_pid.outer.Actual)));
  chassis->forward_speed_Y =
      -sqrtf(2.0f) / 4.0f *
      ((Motor_1_Speed - Motor_3_Speed) * (sinf(chassis->yaw_pid.outer.Actual) -
                                          cosf(chassis->yaw_pid.outer.Actual)) +
       (Motor_2_Speed - Motor_4_Speed) * (cosf(chassis->yaw_pid.outer.Actual) +
                                          sinf(chassis->yaw_pid.outer.Actual)));
  // 获取底盘速度
  chassis->Speed_pid.inner.Actual =
      sqrtf(chassis->forward_speed_X * chassis->forward_speed_X +
            chassis->forward_speed_Y * chassis->forward_speed_Y);

  /************************************************************
   *                  单环PID
   ************************************************************/
  PID_Update(&chassis->Speed_pid.inner, chassis_mode);
  //p_reg->chassis.Speed_pid.inner.OUTMAX = OMNI_SPEED_MAX;
  /************************************************************
   *                  底盘速度解算为通道值
   ************************************************************/
  // chassis->yaw_pid.outer.Actual = p_reg->gimbal.yaw_pid.outer.Actual;
  float sin_ang = sinf(chassis->yaw_pid.outer.Actual);
  float cos_ang = cosf(chassis->yaw_pid.outer.Actual);
  float speed_X =
      chassis->Speed_pid.inner.Out * cosf(chassis->yaw_pid.outer.Target);
  float speed_Y =
      chassis->Speed_pid.inner.Out * sinf(chassis->yaw_pid.outer.Target);

  speed_cal[0] =
      (float)((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y +
              CHASSIS_RADIUS * chassis->target_omega) /
      sqrtf(2) / 2 / PI * 60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
  speed_cal[1] =
      (float)((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y +
              CHASSIS_RADIUS * chassis->target_omega) /
      sqrtf(2) / 2 / PI * 60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
  speed_cal[2] =
      (float)((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y +
              CHASSIS_RADIUS * chassis->target_omega) /
      sqrtf(2) / 2 / PI * 60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
  speed_cal[3] =
      (float)((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y +
              CHASSIS_RADIUS * chassis->target_omega) /
      sqrtf(2) / 2 / PI * 60.0f * GEAR_RATIO_3508 / CHASSIS_RADIUS;
  for(uint8_t i = 0 ;i < 4;i++)
  {
    if(speed_cal[i] > OMNI_3508_CAL_MAX)
    {
      speed_cal[i] = OMNI_3508_CAL_MAX;
    }
    else if(speed_cal[i] < -OMNI_3508_CAL_MAX)
    {
      speed_cal[i] = -OMNI_3508_CAL_MAX;
    }
  }
}
