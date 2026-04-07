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
#include "bsp_Motor.h"

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
    uint8_t rx_data[2 * 4 + 2] = {0};
    static uint32_t last_rx_tick = 0;          // 上次收到数据时的系统滴答
    const uint32_t timeout_tick = 1000;         // 超时阈值，单位 ms（可调整）

    /* Infinite loop */
    for(;;)
    {
        osStatus_t status = osMessageQueueGet(USBRxQueueHandle,&rx_data,NULL,25);
        uint32_t now = osKernelGetTickCount(); // 或 HAL_GetTick()

        if (status == osOK)
        {
            p_reg->gimbal.sentry_state = SENTRY_DISABLED;
            uint8_t *p_byte;
            // 处理数据
            p_byte = &rx_data[1];
            memcpy(&p_reg->gimbal.recvpacket.yaw,p_byte, 4);
            p_byte = &rx_data[5];
            memcpy(&p_reg->gimbal.recvpacket.pitch,p_byte, 4);
            p_byte = NULL;
        } 
        else
        {
            if ((now - last_rx_tick) >= timeout_tick)
            {
                p_reg->gimbal.sentry_state = SENTRY_ENABLED;
                // 云台pitch内环
                p_reg->gimbal.pitch_pid.inner.kp = 20.0f;
                p_reg->gimbal.pitch_pid.inner.ki = 40.0f;
                p_reg->gimbal.pitch_pid.inner.OUTMAX = 5000.0f;
                p_reg->gimbal.pitch_pid.inner.IMAX = 5000.0F;
                // 云台pitch外环
                p_reg->gimbal.pitch_pid.outer.kp = 75.0f;
                p_reg->gimbal.pitch_pid.outer.ki = 30.0f;
                p_reg->gimbal.pitch_pid.outer.OUTMAX = 5000.0f;
                p_reg->gimbal.pitch_pid.outer.IMAX = 5000.0F;
                // 云台yaw内环
                p_reg->gimbal.yaw_pid.inner.kp = 100.0f;
                p_reg->gimbal.yaw_pid.inner.ki = 10.0f;
                p_reg->gimbal.yaw_pid.inner.OUTMAX = 5000.0f;
                p_reg->gimbal.yaw_pid.inner.IMAX = 5000.0F;
                // 云台yaw外环
                p_reg->gimbal.yaw_pid.outer.kp = 15.0f;
                p_reg->gimbal.yaw_pid.outer.ki = 5.0f;
                p_reg->gimbal.yaw_pid.outer.OUTMAX = 500.0f;
                p_reg->gimbal.yaw_pid.outer.IMAX = 1000.0F;
            }
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
    /* USER CODE BEGIN StartPIDTask */
    /* Infinite loop */
    for(;;)
    {
        p_reg->gimbal.pitch_pid.inner.Target = p_reg->gimbal.pitch_pid.outer.Out;
        p_reg->gimbal.pitch_pid.inner.Actual = p_reg->gimbal.pitch_RxData.data2;

        p_reg->gimbal.yaw_pid.inner.Target = p_reg->gimbal.yaw_pid.outer.Out;
        p_reg->gimbal.yaw_pid.inner.Actual = p_reg->gimbal.yaw_RxData.data2;
        uint32_t enter_time = HAL_GetTick();
        PID_Update(&p_reg->gimbal.pitch_pid.inner,gimbal_mode);
        PID_Update(&p_reg->gimbal.yaw_pid.inner,gimbal_mode);
        uint32_t PID_finished_time = HAL_GetTick() - enter_time;
        // ========================将PID结果放在对应ID位置
        p_reg->gimbal.pitch_pid.inner.Out = -(int16_t)p_reg->gimbal.pitch_pid.inner.Out;
        p_reg->TxData.data4 = p_reg->gimbal.pitch_pid.inner.Out;
        p_reg->TxData.data2 = (int16_t)p_reg->gimbal.yaw_pid.inner.Out;
        uint32_t begin_CAN_time = HAL_GetTick();
        CAN_Send(CAN_6020_1,&p_reg->TxData,4);
        uint32_t end_CAN_time = HAL_GetTick() - begin_CAN_time;
        // 每次发送完清零
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

        /***********************************************************************
         *          当视觉没识别到目标，不发送数据
         ***********************************************************************/
        if (p_reg->gimbal.sentry_state == SENTRY_DISABLED)
        {
          p_reg->gimbal.pitch_pid.outer.Target = p_reg->gimbal.recvpacket.pitch;
          if (p_reg->gimbal.pitch_pid.outer.Target > 42)
            p_reg->gimbal.pitch_pid.outer.Target = 42;
          if (p_reg->gimbal.pitch_pid.outer.Target < -42)
            p_reg->gimbal.pitch_pid.outer.Target = -42;

          p_reg->gimbal.yaw_pid.outer.Target = p_reg->gimbal.recvpacket.yaw;
          if (p_reg->gimbal.yaw_pid.outer.Target > 45)
            p_reg->gimbal.yaw_pid.outer.Target = 45;
          if (p_reg->gimbal.yaw_pid.outer.Target < -45)
            p_reg->gimbal.yaw_pid.outer.Target = -45;
        }
        p_reg->gimbal.pitch_pid.outer.Actual = p_reg->gimbal.curr_angle.pitch;
        p_reg->gimbal.yaw_pid.outer.Actual = p_reg->gimbal.curr_angle.yaw;

        PID_Update(&p_reg->gimbal.pitch_pid.outer, gimbal_mode);
        PID_Update(&p_reg->gimbal.yaw_pid.outer, gimbal_mode);

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
void Startbmi088Task(void *argument)
{
    /* USER CODE BEGIN Startbmi088Task */
    float temper = 0;
    float bmi_data[6] = {0};        // AX,AY,AZ,GX,GY,GZ
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    /*************************************************************
     *                  测试
     *************************************************************/
    twoKp = 2.0f * 0.35f;//比例增益  Kp 大：修正更快，但可能抖
    twoKi = 2.0f * 0.02f;//积分增益  Ki 大：能消除长期漂移，但太大可能积累误差过多
    /*************************************************************
     *                  测试
     *************************************************************/
    float *p_bmi_data = bmi_data;
    float *p_temp = &temper;
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(&p_bmi_data[0], &p_bmi_data[3],p_temp);
        MahonyAHRSupdateIMU(q,bmi_data[0],bmi_data[1],bmi_data[2],
                                bmi_data[3],bmi_data[4],bmi_data[5]);
        // /*********************************************************************
        //  *                  云台数据
        //  ********************************************************************/
        p_reg->gimbal.curr_angle.yaw  = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
                                    //* 57.29578f;
        p_reg->gimbal.curr_angle.roll = asinf(2*(q[0]*q[2]-q[3]*q[1]));
                                    //* 57.29578f;
        p_reg->gimbal.curr_angle.pitch = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
                                    //* 57.29578f;
        INS_QuaternionToEuler(q, &p_reg->gimbal.curr_angle.pitch,
                              &p_reg->gimbal.curr_angle.roll, &p_reg->gimbal.curr_angle.yaw);
        p_reg->gimbal.curr_angle.pitch = -p_reg->gimbal.curr_angle.pitch;
       
        /*********************************************************************
         *                  底盘bmi088数据
         ********************************************************************/
        // p_reg->chassis.actual_omega = p_bmi_data[2] * 57.3f;
        // p_reg->chassis.yaw_pid.outer.Actual = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
        //                             * 57.3f;
        // p_reg->chassis.pitch_pid.outer.Actual = asinf(2*(q[0]*q[2]-q[3]*q[1]))
        //                             * 57.3f;
        // p_reg->chassis.roll_pid.outer.Actual  = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        //                             * 57.3f;
        // INS_QuaternionToEuler(q, &p_reg->chassis.pitch_pid.outer.Actual,
        //                       &p_reg->chassis.roll_pid.outer.Actual, &p_reg->chassis.yaw_pid.outer.Actual);

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
    float YAW_MIN = -45.0f, YAW_MAX = 45.0f, PITCH_MIN = -40.0f, PITCH_MAX = 40.0f;
    const float YAW_INIT = 1.0f, PITCH_INIT = 1.0f;

    float start_yaw = YAW_INIT;
    float start_pitch = PITCH_INIT;
    /**********************************************************
     *                  正弦函数版
     **********************************************************/
    const float YAW_CENTER = 0.0f;      // 中心位置（度）
    const float YAW_AMPLITUDE = 45.0f;  // 振幅（度） → 扫描范围 ±45°
    const float PITCH_CENTER = 0.0f;
    const float PITCH_AMPLITUDE = 41.0f;

    const float FREQ_YAW_K = 1.0f;      // 0.5f为慢速，1.0f为快速
    const float FREQ_PITCH_K = 2.0f;    // 1.0f为慢速，2.0f为快速
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
            if (p_reg->gimbal.yaw_pid.outer.Target < YAW_MIN) p_reg->gimbal.yaw_pid.outer.Target = YAW_MIN;
            if (p_reg->gimbal.yaw_pid.outer.Target > YAW_MAX) p_reg->gimbal.yaw_pid.outer.Target = YAW_MAX;
            if (p_reg->gimbal.pitch_pid.outer.Target < PITCH_MIN) p_reg->gimbal.pitch_pid.outer.Target = PITCH_MIN;
            if (p_reg->gimbal.pitch_pid.outer.Target > PITCH_MAX) p_reg->gimbal.pitch_pid.outer.Target = PITCH_MAX;

            // 发送数据
            p_reg->TxData.data1 = (int16_t)(p_reg->gimbal.yaw_pid.outer.Target);
            p_reg->TxData.data2 = (int16_t)(p_reg->gimbal.pitch_pid.outer.Target);
            CAN_Send(CAN_6020_1, &p_reg->TxData, 4);
            // memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
        }
        else
        {
            /*****************************************************
            *   退出哨兵模式：记录当前角度，下次从这里继续
            *****************************************************/
            start_yaw = p_reg->gimbal.yaw_pid.outer.Target;
            start_pitch = p_reg->gimbal.pitch_pid.outer.Target;

            // 边界保护
            if (start_yaw < YAW_MIN) start_yaw = YAW_MIN;
            if (start_yaw > YAW_MAX) start_yaw = YAW_MAX;
            if (start_pitch < PITCH_MIN) start_pitch = PITCH_MIN;
            if (start_pitch > PITCH_MAX) start_pitch = PITCH_MAX;

            start_time = HAL_GetTick();  // 重置时间，确保下次扫描从头开始

            // 重置 first_resume，下次进入哨兵模式时会先读取 start_yaw/start_pitch
            static uint8_t *p_first_resume = NULL;
            if (p_first_resume == NULL)
            {
                // 这里用指针是为了访问 else 分支里的静态变量，更简单的做法是把 first_resume 提到外面
                // 为了简化，我们直接把 first_resume 提到任务开头作为全局静态变量
            }
        }
        osDelay(SCAN_DELAY);
    }
}

/*********************************************************************************
 *                          工具函数
 *********************************************************************************/

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
    float yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

    //弧度转角度
    *roll_deg = roll * 57.29578f;
    *pitch_deg = pitch * 57.29578f;
    *yaw_deg = yaw * 57.29578f;
}
