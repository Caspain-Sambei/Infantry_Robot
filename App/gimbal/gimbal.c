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

    /* Infinite loop */
    for(;;)
    {
        osStatus_t status = osMessageQueueGet(USBRxQueueHandle,&rx_data,NULL,osWaitForever);

        if (status == osOK)
        {
            p_reg->gimbal.sentry_state = SENTRY_ENABLED;
            uint8_t *p_byte;
            // 处理数据
            p_byte = &rx_data[1];
            memcpy(&p_reg->gimbal.recvpacket.pitch,p_byte, 4);
            p_byte = &rx_data[5];
            memcpy(&p_reg->gimbal.recvpacket.yaw,p_byte, 4);
            p_byte = NULL;
        }
        else
            p_reg->gimbal.sentry_state = SENTRY_DISABLED;
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
        p_reg->gimbal.pitch_pid.inner.Target = p_reg->gimbal.pitch_pid.inner.Out;
        p_reg->gimbal.pitch_pid.inner.Actual = p_reg->gimbal.pitch_RxData.data2;

        p_reg->gimbal.yaw_pid.inner.Target = p_reg->gimbal.yaw_pid.inner.Out;
        p_reg->gimbal.yaw_pid.inner.Actual = p_reg->gimbal.yaw_RxData.data2;

        PID_Update(&p_reg->gimbal.pitch_pid.inner,gimbal_mode);
        PID_Update(&p_reg->gimbal.yaw_pid.inner,gimbal_mode);

        // ========================将PID结果放在对应ID位置
        p_reg->TxData.data1 = (int16_t)p_reg->gimbal.pitch_pid.outer.Out;
        p_reg->TxData.data2 = (int16_t)p_reg->gimbal.yaw_pid.outer.Out;

        CAN_Send(CAN_6020_1,&p_reg->TxData,4);
        // 每次发送完清零
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
        osDelay(1);
    }
    /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_gimbal_exPIDTask */
/**
* @brief Function implementing the gimbal_exPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_exPIDTask */
void gimbal_exPIDTask(void *argument)
{
    /* USER CODE BEGIN StartexPIDTask */
    /* Infinite loop */
    for(;;)
    {
        p_reg->gimbal.pitch_pid.outer.Target = p_reg->gimbal.recvpacket.pitch;
            // 软件限位，限定在第一和第四象限
            if (p_reg->gimbal.pitch_pid.outer.Target >= 270 && p_reg->gimbal.pitch_pid.outer.Target <= 360)
                p_reg->gimbal.pitch_pid.outer.Target = -90;
            if (p_reg->gimbal.pitch_pid.outer.Target >= 90)
                p_reg->gimbal.pitch_pid.outer.Target = 90;
            if (p_reg->gimbal.pitch_pid.outer.Target <= -90)
                p_reg->gimbal.pitch_pid.outer.Target = -90;
        p_reg->gimbal.pitch_pid.outer.Actual = p_reg->gimbal.curr_angle.pitch;

        p_reg->gimbal.yaw_pid.outer.Target = p_reg->gimbal.recvpacket.yaw;
        p_reg->gimbal.yaw_pid.outer.Actual = p_reg->gimbal.curr_angle.yaw;

        PID_Update(&p_reg->gimbal.pitch_pid.outer,gimbal_mode);
        PID_Update(&p_reg->gimbal.yaw_pid.outer,gimbal_mode);

        osDelay(10);
    }
    /* USER CODE END StartexPIDTask */
}

/***************************************************************************
 *					        BMI088和四元解算
 *					        向视觉发送数据
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
    // SENDPACKET send_data = {
    //     .pitch = 0.0f,
    //     .yaw = 0.0f,
    //     .roll = 0.0f,
    // };
    // SENDPACKET *p_send_data = &send_data;

    float *p_bmi_data = bmi_data;
    float *p_temp = &temper;
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(&p_bmi_data[0], &p_bmi_data[3],p_temp);
        MahonyAHRSupdateIMU(q,bmi_data[3],bmi_data[4],bmi_data[5],
                                 bmi_data[0],bmi_data[1],bmi_data[2]);
        /*********************************************************************
         *                  云台数据
         ********************************************************************/
        // p_reg->gimbal.curr_angle.yaw  = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
        //                             * 57.3f;
        // p_reg->gimbal.curr_angle.pitch = asinf(2*(q[0]*q[2]-q[3]*q[1]))
        //                             * 57.3f;
        // p_reg->gimbal.curr_angle.roll  = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        //                             * 57.3f;
        /*********************************************************************
         *                  底盘bmi088数据
         ********************************************************************/
        // 获取底盘当前的角速度
        p_reg->chassis.actual_omega = p_bmi_data[2] * 57.3f;
        p_reg->chassis.yaw_pid.outer.Actual = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
                                    * 57.3f;
        p_reg->chassis.pitch_pid.outer.Actual = asinf(2*(q[0]*q[2]-q[3]*q[1]))
                                    * 57.3f;
        p_reg->chassis.roll_pid.outer.Actual  = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))
                                    * 57.3f;
        // 向视觉发送数据
        p_reg->gimbal.sendpacket.pitch = p_reg->gimbal.curr_angle.pitch;
        p_reg->gimbal.sendpacket.yaw = p_reg->gimbal.curr_angle.yaw;
        p_reg->gimbal.sendpacket.roll = p_reg->gimbal.curr_angle.roll;

        USB_Send(&p_reg->gimbal.sendpacket);

        osDelay(10);
    }
    /* USER CODE END Startbmi088Task */
}

/***************************************************************************
 *			哨兵模式:云台扫描正前方上下左右90°的正方形平面,假定没检测到时视觉不发送消息
 **************************************************************************/
/* USER CODE BEGIN Header_StartSentry_modeTask */
/**
* @brief Function implementing the sentry_mode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSentry_modeTask */
void StartSentry_modeTask(void* argument)
{
    /* USER CODE BEGIN StartSentry_modeTask */
    /***************************************************************
     * 俯仰，偏航各90°的正方形内沿8字形扫描，分辨率为1°，设置对应的扫描次数为90
     ***************************************************************/
    const float scan_angle = 1.0f; // 扫描步距
    const uint16_t SCAN_DELAY = 100; // 扫描步长延时
    // 次数 * 分辨率 = 90
    #define F_EPSILON   0.3f
    float YAW_MIN = -45.0f, YAW_MAX = 45.0f, PITCH_MIN = -45.0f, PITCH_MAX = 45.0f; // 电机限位
    // 初始角度（右上角45°）
    const float YAW_INIT = 45.0f,PITCH_INIT = 45.0f;
    // 首次进入哨兵模式从初始角度开始。
    float start_yaw = 0.0f;
    float start_pitch = 0.0f;
    // 首次是否从中心开始
    uint8_t start_from_center = 1;

    /* Infinite loop */
    for(;;)
    {
        if (p_reg->gimbal.sentry_state == SENTRY_ENABLED)
        {
            /*****************************************************
            *					首次进入
            *****************************************************/
            if (start_from_center == 1)
            {
                start_yaw = YAW_INIT;
                start_pitch = PITCH_INIT;
                // 下次从丢失目标时的位置继续扫描
                start_from_center = 0;
                p_reg->gimbal.yaw_pid.outer.Target = start_yaw;
                p_reg->gimbal.pitch_pid.outer.Target = start_pitch;
            }
            else
            {
                p_reg->gimbal.yaw_pid.outer.Target = start_yaw;
                p_reg->gimbal.pitch_pid.outer.Target = start_pitch;
                /*****************************************************
                *					顶点情况
                *****************************************************/
                // 左上顶点
                if (fabsf(p_reg->gimbal.pitch_pid.outer.Target - PITCH_MAX) < F_EPSILON
                    && fabsf(p_reg->gimbal.yaw_pid.outer.Target - YAW_MIN) < F_EPSILON)
                {
                    p_reg->gimbal.yaw_pid.outer.Target += scan_angle;
                }
                // 右上顶点
                else if (fabsf(p_reg->gimbal.pitch_pid.outer.Target - PITCH_MAX) < F_EPSILON
                    && fabsf(p_reg->gimbal.yaw_pid.outer.Target - YAW_MAX) < F_EPSILON)
                {
                    p_reg->gimbal.yaw_pid.outer.Target -= scan_angle;
                    p_reg->gimbal.pitch_pid.outer.Target -= scan_angle;
                }
                // 左下顶点
                else if (fabsf(p_reg->gimbal.pitch_pid.outer.Target - PITCH_MIN) < F_EPSILON
                    && fabsf(p_reg->gimbal.yaw_pid.outer.Target - YAW_MIN) < F_EPSILON)
                {
                    p_reg->gimbal.yaw_pid.outer.Target += scan_angle;
                }
                // 右下顶点
                else if (fabsf(p_reg->gimbal.pitch_pid.outer.Target - PITCH_MIN) < F_EPSILON
                    && fabsf(p_reg->gimbal.yaw_pid.outer.Target - YAW_MAX) < F_EPSILON)
                {
                    p_reg->gimbal.yaw_pid.outer.Target += scan_angle;
                    p_reg->gimbal.pitch_pid.outer.Target += scan_angle;
                }
                /*****************************************************
                *					转动部分
                *****************************************************/
                // 当云台处于最上和最下时yaw都是从左往右扫描
                if ((fabsf(p_reg->gimbal.pitch_pid.outer.Target - 45.0f) < F_EPSILON
                    ||fabsf(p_reg->gimbal.pitch_pid.outer.Target - -45.0f) < F_EPSILON)
                    && p_reg->gimbal.yaw_pid.outer.Target < YAW_MAX
                    && p_reg->gimbal.yaw_pid.outer.Target > YAW_MIN)
                {
                    p_reg->gimbal.yaw_pid.outer.Target += scan_angle;
                }
                else // 否则就是处在两条对角线上
                {
                    // 第二象限或第四象限
                    if ((p_reg->gimbal.pitch_pid.outer.Target > 0.0f
                            && p_reg->gimbal.pitch_pid.outer.Target < PITCH_MAX
                            && p_reg->gimbal.yaw_pid.outer.Target < 0.0f
                            && p_reg->gimbal.yaw_pid.outer.Target > YAW_MIN)
                        || (p_reg->gimbal.pitch_pid.outer.Target < 0.0f
                            && p_reg->gimbal.pitch_pid.outer.Target > PITCH_MIN
                            && p_reg->gimbal.yaw_pid.outer.Target > 0.0f
                            && p_reg->gimbal.yaw_pid.outer.Target < YAW_MAX))
                    {
                        p_reg->gimbal.yaw_pid.outer.Target -= scan_angle;
                        p_reg->gimbal.pitch_pid.outer.Target += scan_angle;
                        // 判断是否在原点
                        if (fabsf(p_reg->gimbal.yaw_pid.outer.Target - 0.0f) < F_EPSILON
                            && fabsf(p_reg->gimbal.pitch_pid.outer.Target - 0.0f) < F_EPSILON)
                        {
                            p_reg->gimbal.yaw_pid.outer.Target -= scan_angle;
                            p_reg->gimbal.pitch_pid.outer.Target += scan_angle;
                        }
                    }
                    // 第一象限或第三象限
                    else if ((p_reg->gimbal.pitch_pid.outer.Target > 0.0f
                            && p_reg->gimbal.pitch_pid.outer.Target < PITCH_MAX
                            && p_reg->gimbal.yaw_pid.outer.Target > 0.0f
                            && p_reg->gimbal.yaw_pid.outer.Target < YAW_MAX)
                        || (p_reg->gimbal.pitch_pid.outer.Target < 0.0f
                            && p_reg->gimbal.pitch_pid.outer.Target > PITCH_MIN
                            && p_reg->gimbal.yaw_pid.outer.Target < 0.0f
                            && p_reg->gimbal.yaw_pid.outer.Target > YAW_MIN))
                    {
                        p_reg->gimbal.yaw_pid.outer.Target -= scan_angle;
                        p_reg->gimbal.pitch_pid.outer.Target -= scan_angle;
                        // 判断是否在原点
                        if (fabsf(p_reg->gimbal.yaw_pid.outer.Target - 0.0f) < F_EPSILON
                            && fabsf(p_reg->gimbal.pitch_pid.outer.Target - 0.0f) < F_EPSILON)
                        {
                            p_reg->gimbal.yaw_pid.outer.Target -= scan_angle;
                            p_reg->gimbal.pitch_pid.outer.Target -= scan_angle;
                        }
                    }
                }
                // 边界保护
                if (p_reg->gimbal.yaw_pid.outer.Target < YAW_MIN) p_reg->gimbal.yaw_pid.outer.Target = YAW_MIN;
                if (p_reg->gimbal.yaw_pid.outer.Target > YAW_MAX) p_reg->gimbal.yaw_pid.outer.Target = YAW_MAX;
                if (p_reg->gimbal.pitch_pid.outer.Target < PITCH_MIN) p_reg->gimbal.pitch_pid.outer.Target = PITCH_MIN;
                if (p_reg->gimbal.pitch_pid.outer.Target > PITCH_MAX) p_reg->gimbal.pitch_pid.outer.Target = PITCH_MAX;
                // 共用发送部分
                p_reg->TxData.data1 = (int16_t)(p_reg->gimbal.yaw_pid.outer.Target);
                p_reg->TxData.data2 = (int16_t)(p_reg->gimbal.pitch_pid.outer.Target);
                CAN_Send(CAN_6020_1, &p_reg->TxData, 8);
                memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
            }
        }
        else
        {
            // 云台识别到目标，自稳
            // 记录当前云台角度，作为扫描恢复点
            start_yaw = p_reg->gimbal.yaw_pid.outer.Target;
            start_pitch = p_reg->gimbal.pitch_pid.outer.Target;

            if (start_yaw < YAW_MIN) start_yaw = YAW_MIN;
            if (start_yaw > YAW_MAX) start_yaw = YAW_MAX;
            if (start_pitch < PITCH_MIN) start_pitch = PITCH_MIN;
            if (start_pitch > PITCH_MAX) start_pitch = PITCH_MAX;

            start_from_center = 0;
        }
        osDelay(SCAN_DELAY);
    }
    /* USER CODE END StartSentry_modeTask */
}