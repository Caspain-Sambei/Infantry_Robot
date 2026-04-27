//
// Created by 18796 on 2026/2/25.
//
#include "gimbal.h"
#include <tgmath.h>
#include "reg.h"
#include "cmsis_os2.h"
#include "usbd_conf.h"
#include "MyUSB.h"
#include "usb_device.h"
#include "PID.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "MyCAN.h"
#include <math.h>
#include "bsp_delay.h"
#include "bsp_Motor.h"
#include "Filter.h"
#include "KalmanFilter.h"

/***************************************************************************
 *								USB通信
 **************************************************************************/

/* USER CODE BEGIN Header_StartUSB_RxTask */
/**
* @brief Function implementing the USB_RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSB_RxTask */
void StartUSB_RxTask(void *argument)
{
    /* USER CODE BEGIN StartUSB_Rx */
    uint8_t rx_data[3 * 4 + 2] = {0};
    static uint32_t last_rx_tick = 0;          // 上次收到数据时的系统滴答
    const uint32_t timeout_tick = 200;         // 丢失目标超时阈值，单位 ms

    /* Infinite loop */
    for(;;)
    {
        osStatus_t status = osMessageQueueGet(USBRxQueueHandle,&rx_data,NULL,25);
        uint32_t now = osKernelGetTickCount();
        static uint8_t tracked_flag = 0;

        if (status == osOK)
        {
            if (tracked_flag == 0)
            {
                // 第一次识别到目标则积分清零
                PID_Clear(&p_reg->gimbal.pitch_pid.inner);
                PID_Clear(&p_reg->gimbal.pitch_pid.outer);
                PID_Clear(&p_reg->gimbal.yaw_pid.inner);
                PID_Clear(&p_reg->gimbal.yaw_pid.outer);
            }
            tracked_flag = 1;

            p_reg->gimbal.sentry_state = SENTRY_DISABLED;
            uint8_t *p_byte;
            // 处理数据
            p_byte = &rx_data[1];
            memcpy(&p_reg->gimbal.recvpacket.pitch,p_byte, 4);
            p_byte = &rx_data[5];
            memcpy(&p_reg->gimbal.recvpacket.yaw,p_byte, 4);
            p_byte = NULL;
            // 保护
            if (p_reg->gimbal.recvpacket.pitch > 42)
                p_reg->gimbal.recvpacket.pitch = 42;
            if (p_reg->gimbal.recvpacket.pitch < -42)
                p_reg->gimbal.recvpacket.pitch = -42;
            if (p_reg->gimbal.recvpacket.yaw > 45)
                p_reg->gimbal.recvpacket.yaw = 45;
            if (p_reg->gimbal.recvpacket.yaw < -45)
                p_reg->gimbal.recvpacket.yaw = -45;
            // 新增一阶低通滤波
            // Pass_Filter(&p_reg->gimbal.recvpacket.pitch,0.5f);
            // Pass_Filter(&p_reg->gimbal.recvpacket.yaw,0.5f);

            last_rx_tick = now;
        }
        else if (now - last_rx_tick >= timeout_tick )
        {
            p_reg->gimbal.sentry_state = SENTRY_ENABLED;
            // 尝试瞬间积分清零
            PID_Clear(&p_reg->gimbal.pitch_pid.inner);
            PID_Clear(&p_reg->gimbal.pitch_pid.outer);
            PID_Clear(&p_reg->gimbal.yaw_pid.inner);
            PID_Clear(&p_reg->gimbal.yaw_pid.outer);
            // 判断是否是重复数据
            p_reg->gimbal.recvpacket.pitch = 0.0f;
            p_reg->gimbal.recvpacket.yaw = 0.0f;

            tracked_flag = 0;
        }
        osDelay(1);
    }
    /* USER CODE END StartUSB_Rx */
}


/***************************************************************************
 *					云台PID，最大输出为500，积分限幅为100
 **************************************************************************/
/* USER CODE BEGIN Header_gimbal_inPIDTask */
/**
  * @brief  Function implementing the gimbal_inPID thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_gimbal_inPIDTask */
void gimbal_inPIDTask(void *argument)
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    uint16_t cnt = 0;
    /* USER CODE BEGIN StartPIDTask */
    /* Infinite loop */
    for(;;)
    {
        /***************************************************************************
         *					        外环
         **************************************************************************/
        if (p_reg->gimbal.sentry_state == SENTRY_DISABLED)
        {
            p_reg->gimbal.pitch_pid.outer.Target = p_reg->gimbal.recvpacket.pitch;
            p_reg->gimbal.yaw_pid.outer.Target = p_reg->gimbal.recvpacket.yaw;

            if (p_reg->gimbal.pitch_pid.outer.Target > 42) { p_reg->gimbal.pitch_pid.outer.Target = 42; }
            if (p_reg->gimbal.pitch_pid.outer.Target < -42) { p_reg->gimbal.pitch_pid.outer.Target = -42; }
            if (p_reg->gimbal.yaw_pid.outer.Target > 45) { p_reg->gimbal.yaw_pid.outer.Target = 45; }
            if (p_reg->gimbal.yaw_pid.outer.Target < -45) { p_reg->gimbal.yaw_pid.outer.Target = -45; }
        }
        cnt = HAL_GetTick();
        p_reg->gimbal.pitch_pid.outer.Actual = p_reg->gimbal.curr_angle.pitch;
        p_reg->gimbal.yaw_pid.outer.Actual = p_reg->gimbal.curr_angle.yaw;

        PID_Update(&p_reg->gimbal.pitch_pid.outer, GIMBAL_MODE);
        PID_Update(&p_reg->gimbal.yaw_pid.outer, GIMBAL_MODE);

        /***************************************************************************
         *					        内环
         **************************************************************************/
        p_reg->gimbal.pitch_pid.inner.Target = p_reg->gimbal.pitch_pid.outer.Out;
        p_reg->gimbal.pitch_pid.inner.Actual = p_reg->gimbal.pitch_RxData.data2;
        p_reg->gimbal.yaw_pid.inner.Target = p_reg->gimbal.yaw_pid.outer.Out;
        p_reg->gimbal.yaw_pid.inner.Actual = p_reg->gimbal.yaw_RxData.data2;

        PID_Update(&p_reg->gimbal.pitch_pid.inner,GIMBAL_MODE);
        PID_Update(&p_reg->gimbal.yaw_pid.inner,GIMBAL_MODE);

        p_reg->TxData.data4 = (int16_t)p_reg->gimbal.pitch_pid.inner.Out;
        p_reg->TxData.data2 = (int16_t)p_reg->gimbal.yaw_pid.inner.Out;
        cnt = HAL_GetTick() - cnt;
        CAN_Send(CAN_6020_1, &p_reg->TxData, 4);
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

        osDelay(1);
    }
    /* USER CODE END StartPIDTask */
}

/***************************************************************************
 *					        BMI088和四元解算
 *					        向视觉发送三个欧拉角
 **************************************************************************/
/* USER CODE BEGIN Header_Startbmi088Task */
/**
* @brief Function implementing the bmi088Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startbmi088Task */
void Startbmi088Task(void* argument)
{
    /* USER CODE BEGIN Startbmi088Task */
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    // twoKp = 2.0f * 0.5f;//比例增益  Kp 大：修正更快，但可能抖
    // twoKi = 2.0f * 0.0f;//积分增益  Ki 大：能消除长期漂移，但太大可能积累误差过多
    static EKF_IMU ekf_imu = {0};
    EKF_IMU_Init(&ekf_imu, q,p_reg->gyro_bias);

    /* Infinite loop */
    for (;;)
    {
        BMI088_read(&p_reg->bmi088_real_data.gyro[0], &p_reg->bmi088_real_data.accel[0], NULL);
        /*********************************************************************
         *                           MahonyAHRS
         ********************************************************************/
        // 陀螺仪减去零偏，得到校准数据
        // for (uint8_t i = 0; i < 3; i++)
        // {
        //     p_reg->bmi088_real_data.gyro[i] -= p_reg->gyro_bias[i];
        //     p_reg->bmi088_real_data.accel[i] -= p_reg->accel_bias[i];
        // }
        // 更新四元数
        // MahonyAHRSupdateIMU(q, p_reg->bmi088_real_data.gyro[0], p_reg->bmi088_real_data.gyro[1],
        //                     p_reg->bmi088_real_data.gyro[2],
        //                     p_reg->bmi088_real_data.accel[0], p_reg->bmi088_real_data.accel[1],
        //                     p_reg->bmi088_real_data.accel[2]);
        // p_reg->bmi088_real_data.gyro[0] *= 0.0174533f;
        // p_reg->bmi088_real_data.gyro[1] *= 0.0174533f;
        // p_reg->bmi088_real_data.gyro[2] *= 0.0174533f;
        /*********************************************************************
         *                           EKF
         ********************************************************************/
        EKF_IMU_Update(&ekf_imu, &p_reg->bmi088_real_data.gyro[0], &p_reg->bmi088_real_data.accel[0],0.001f);

        /*********************************************************************
         *                  云台数据 MahonyAHRS + INS
         ********************************************************************/
        // p_reg->gimbal.curr_angle.yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
        // p_reg->gimbal.curr_angle.roll = asinf(2 * (q[0] * q[2] - q[3] * q[1]));
        // p_reg->gimbal.curr_angle.pitch = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
        //
        // INS_QuaternionToEuler(q, &p_reg->gimbal.curr_angle.pitch,
        //                       &p_reg->gimbal.curr_angle.roll, &p_reg->gimbal.curr_angle.yaw);
        //
        // // Pass_Filter(&p_reg->gimbal.curr_angle.pitch, 0.75f);
        // // Pass_Filter(&p_reg->gimbal.curr_angle.yaw, 0.85f);
        // p_reg->gimbal.curr_angle.pitch =
        //     kalman_filter_1D(p_reg->gimbal.curr_angle.pitch, 0.0005f, 0.001f, 0.0f, 1.0f);
        //
        // p_reg->gimbal.curr_angle.yaw =
        //     kalman_filter_1D(p_reg->gimbal.curr_angle.yaw, 0.0005f, 0.001f, 0.0f, 1.0f);

        /*********************************************************************
         *                  云台数据 EKF
         ********************************************************************/
        EKF_IMU_GetEuler(&ekf_imu,&p_reg->gimbal.curr_angle.pitch,&p_reg->gimbal.curr_angle.roll, &p_reg->gimbal.curr_angle.yaw);
        p_reg->gimbal.curr_angle.pitch *= 57.29578f;
        p_reg->gimbal.curr_angle.yaw *= 57.29578f;
        p_reg->gimbal.curr_angle.roll *= 57.29578f;
        Pass_Filter(&p_reg->gimbal.curr_angle.yaw, 0.95f);

        /*********************************************************************
         *                  底盘bmi088数据 MahonyAHRS + INS
         ********************************************************************/
        // p_reg->chassis.yaw_pid.outer.Actual = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
        // INS_QuaternionToEuler(q, NULL,NULL, &p_reg->chassis.yaw_pid.outer.Actual);
        // Pass_Filter(&p_reg->gimbal.curr_angle.yaw, 0.85f);

        /*********************************************************************
         *                  底盘bmi088数据 EKF
         ********************************************************************/
        // EKF_IMU_GetEuler(&ekf_imu,NULL,NULL, &p_reg->gimbal.curr_angle.yaw);
        // p_reg->gimbal.curr_angle.yaw *= 57.29578f;

        /*********************************************************************
         *                  向视觉发送数据
         ********************************************************************/
        p_reg->gimbal.sendpacket.pitch = p_reg->gimbal.curr_angle.pitch;
        p_reg->gimbal.sendpacket.yaw = p_reg->gimbal.curr_angle.yaw;
        p_reg->gimbal.sendpacket.roll = p_reg->gimbal.curr_angle.roll;

        USB_Send(&p_reg->gimbal.sendpacket);
        osDelay(1);
    }
    /* USER CODE END Startbmi088Task */
}
/***************************************************************************
 *			哨兵模式:云台扫描正前方上下左右90°的正方形平面,假定没检测到时视觉不发送消息
 **************************************************************************/
void StartSentry_modeTask(void* argument)
{
    const uint16_t SCAN_DELAY = 10;
    #define F_EPSILON   0.3f // 放宽阈值，避免浮点误差
    float YAW_MIN = -42.0f, YAW_MAX = 42.0f, PITCH_MIN = -40.0f, PITCH_MAX = 40.0f;
    const float YAW_INIT = 1.0f, PITCH_INIT = 1.0f;

    float start_yaw = YAW_INIT;
    float start_pitch = PITCH_INIT;
    /**********************************************************
     *                  正弦函数版
     **********************************************************/
    const float YAW_CENTER = 0.0f;      // 中心位置（度）
    const float YAW_AMPLITUDE = 43.0f;  // 振幅（度） → 扫描范围 ±45°
    const float PITCH_CENTER = 0.0f;
    const float PITCH_AMPLITUDE = 40.0f;

    const float FREQ_YAW_K = 1.5f;      // 0.5f为慢速，1.0f为快速
    const float FREQ_PITCH_K = 6.0f;    // 2.0f为慢速，6.0f为快速
    const float FREQ = 0.2f;            // 摆动频率（Hz），即每秒摆多少个周期

    uint32_t start_time = HAL_GetTick();

    for(;;)
    {
        if (p_reg->gimbal.sentry_state == SENTRY_ENABLED)
        {
            // 计算经过的时间（秒）
            uint32_t now = HAL_GetTick();

            float t = (float)(now - start_time) / 1000.0f;
            // 正弦运动：角度 = 中心 + 振幅 * sin(2π * 频率 * 时间)
            float yaw_target   = YAW_CENTER + YAW_AMPLITUDE * sinf(2.0f * PI * (FREQ * FREQ_YAW_K) * t);
            // PITCH的周期是YAW的两倍
            float pitch_target = PITCH_CENTER + PITCH_AMPLITUDE * sinf(2.0f * PI * (FREQ * FREQ_PITCH_K) * t);
            // 限幅
            if (yaw_target > YAW_MAX) yaw_target = YAW_MAX;
            if (yaw_target < YAW_MIN) yaw_target = YAW_MIN;
            if (pitch_target > PITCH_MAX) pitch_target = PITCH_MAX;
            if (pitch_target < PITCH_MIN) pitch_target = PITCH_MIN;

            p_reg->gimbal.yaw_pid.outer.Target = yaw_target;
            p_reg->gimbal.pitch_pid.outer.Target = pitch_target;

            // 全局变量的边界保护
            if (p_reg->gimbal.yaw_pid.outer.Target < YAW_MIN) {p_reg->gimbal.yaw_pid.outer.Target = YAW_MIN;}
            if (p_reg->gimbal.yaw_pid.outer.Target > YAW_MAX) {p_reg->gimbal.yaw_pid.outer.Target = YAW_MAX;}
            if (p_reg->gimbal.pitch_pid.outer.Target < PITCH_MIN) {p_reg->gimbal.pitch_pid.outer.Target = PITCH_MIN;}
            if (p_reg->gimbal.pitch_pid.outer.Target > PITCH_MAX) {p_reg->gimbal.pitch_pid.outer.Target = PITCH_MAX;}
        }
        else
        {
            /*****************************************************
            *   退出哨兵模式：记录当前角度，下次从这里继续
            *****************************************************/
            start_yaw = p_reg->gimbal.yaw_pid.outer.Target;
            start_pitch = p_reg->gimbal.pitch_pid.outer.Target;

            // 边界保护
            if (start_yaw < YAW_MIN) {start_yaw = YAW_MIN;}
            if (start_yaw > YAW_MAX) {start_yaw = YAW_MAX;}
            if (start_pitch < PITCH_MIN) {start_pitch = PITCH_MIN;}
            if (start_pitch > PITCH_MAX) {start_pitch = PITCH_MAX;}

            start_time = HAL_GetTick();  // 重置时间，确保下次扫描从头开始

            // 重置 first_resume，下次进入哨兵模式时会先读取 start_yaw/start_pitch
            static uint8_t *p_first_resume = NULL;
            if (p_first_resume == NULL)
            {
                // 这里用指针是为了访问 else 分支里的静态变量，更简单的做法是把 first_resume 提到外面
            }
        }
        osDelay(SCAN_DELAY);
    }
}

/*********************************************************************************
 *                          工具函数
 *********************************************************************************/
/**
 * @brief 计算陀螺仪零偏
 *
 * 通过采集多个样本数据并计算平均值来确定陀螺仪的零偏值。
 * 在采集前会丢弃部分不稳定的初始读数以提高校准精度。
 *
 * @param samples 采集的样本数量，用于计算平均值
 * @param gyro_bias 输出参数，指向包含三个元素的数组，分别存储X、Y、Z轴的零偏值
 */
void calibrate_gyro_bias(uint16_t samples,float *gyro_bias,float *accel_bias)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    float sum_accel_x = 0,sum_accel_y = 0,sum_accel_z = 0;
    float temp_data[3],temp_accel_date[3]; // 临时存储单次读取数据
    // 丢弃前几次可能不稳定的读数
    for (int i = 0; i < 50; i++)
    {
        delay_ms(1);
    }

    // 采集 samples 个样本并求平均
    for (int i = 0; i < samples; i++)
    {
        BMI088_read(temp_data, temp_accel_date, NULL);
        sum_x += temp_data[0];
        sum_y += temp_data[1];
        sum_z += temp_data[2];

        // sum_accel_x += temp_accel_date[0];
        // sum_accel_y += temp_accel_date[1];
        // sum_accel_z += temp_accel_date[2];
        delay_ms(1); // 根据采样率调整延时
    }

    gyro_bias[0] = sum_x / (float)samples;
    gyro_bias[1] = sum_y / (float)samples;
    gyro_bias[2] = sum_z / (float)samples;

    // accel_bias[0] = (sum_accel_x / (float)samples);
    // accel_bias[1] = sum_accel_y / (float)samples;
    // accel_bias[2] = (sum_accel_z / (float)samples) - 0.1f;
}
