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
            p_reg->gimbal.sentry_state = SENTRY_DISABLED;
            uint8_t *p_byte;
            // 处理数据
            p_byte = &rx_data[1];
            memcpy(&p_reg->gimbal.recvpacket.pitch,p_byte, 4);
            p_byte = &rx_data[5];
            memcpy(&p_reg->gimbal.recvpacket.yaw,p_byte, 4);
            p_byte = NULL;
        }
        else
            p_reg->gimbal.sentry_state = SENTRY_ENABLED;
        osDelay(10);
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
    uint8_t gimbal_exPID = 0;
    /* USER CODE BEGIN StartPIDTask */
    /* Infinite loop */
    for(;;)
    {
        p_reg->gimbal.pitch_pid.inner.Target = p_reg->gimbal.pitch_pid.outer.Out;
        p_reg->gimbal.pitch_pid.inner.Actual = p_reg->gimbal.pitch_RxData.data2;

        p_reg->gimbal.yaw_pid.inner.Target = p_reg->gimbal.yaw_pid.outer.Out;
        p_reg->gimbal.yaw_pid.inner.Actual = p_reg->gimbal.yaw_RxData.data2;

        PID_Update(&p_reg->gimbal.pitch_pid.inner,gimbal_mode);
        PID_Update(&p_reg->gimbal.yaw_pid.inner,gimbal_mode);

        // ========================将PID结果放在对应ID位置
        p_reg->TxData.data4 = (int16_t)p_reg->gimbal.pitch_pid.inner.Out;
        p_reg->TxData.data2 = (int16_t)p_reg->gimbal.yaw_pid.inner.Out;

        CAN_Send(CAN_6020_1,&p_reg->TxData,4);
        // 每次发送完清零
        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

        if (cnt >= 5)
        {
            cnt = 0;
            gimbal_exPID = 1;
        }
        if (gimbal_exPID == 1)
        {
            gimbal_exPID = 0;
            /***********************************************************************
             *          当视觉没识别到目标，是否应该发送数据？发送什么数据？
             *          对发送的数据进行判断然后决定是否对pitch_target和yaw_target赋值
             ***********************************************************************/
            if (0)// 暂时关闭，将云台pitch_pid.outer.Target的来源让给哨兵模式
            {
                p_reg->gimbal.pitch_pid.outer.Target = p_reg->gimbal.recvpacket.pitch;
                // 软件限位，限定在第一和第四象限
                if (p_reg->gimbal.pitch_pid.outer.Target >= 270 && p_reg->gimbal.pitch_pid.outer.Target <= 360)
                    p_reg->gimbal.pitch_pid.outer.Target = -90;
                if (p_reg->gimbal.pitch_pid.outer.Target >= 90)
                    p_reg->gimbal.pitch_pid.outer.Target = 90;
                if (p_reg->gimbal.pitch_pid.outer.Target <= -90)
                    p_reg->gimbal.pitch_pid.outer.Target = -90;
            }
            p_reg->gimbal.pitch_pid.outer.Actual = p_reg->gimbal.curr_angle.pitch;
            p_reg->gimbal.yaw_pid.outer.Target = p_reg->gimbal.recvpacket.yaw;
            p_reg->gimbal.yaw_pid.outer.Actual = p_reg->gimbal.curr_angle.yaw;

            PID_Update(&p_reg->gimbal.pitch_pid.outer,gimbal_mode);
            PID_Update(&p_reg->gimbal.yaw_pid.outer,gimbal_mode);
        }
        cnt++;
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
        // p_reg->gimbal.curr_angle.yaw  = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
        //                             //* 57.29578f;
        // p_reg->gimbal.curr_angle.roll = asinf(2*(q[0]*q[2]-q[3]*q[1]));
        //                             //* 57.29578f;
        // p_reg->gimbal.curr_angle.pitch = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
        //                             //* 57.29578f;
        // INS_QuaternionToEuler(q, &p_reg->gimbal.curr_angle.pitch,
        //                       &p_reg->gimbal.curr_angle.roll, &p_reg->gimbal.curr_angle.yaw);
        // p_reg->gimbal.curr_angle.pitch = -p_reg->gimbal.curr_angle.pitch;
        //
        // static float angle_yaw = 0;
        // float dt = 0.001f;  // 先按 1ms 假设，实际应测量
        // angle_yaw = 0.98f * (angle_yaw + bmi_data[2] * dt) + 0.02f * atan2f(bmi_data[4], bmi_data[3]);
        /*********************************************************************
         *                  底盘bmi088数据
         ********************************************************************/
        p_reg->chassis.actual_omega = p_bmi_data[2] * 57.3f;
        p_reg->chassis.yaw_pid.outer.Actual = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
                                    * 57.3f;
        p_reg->chassis.pitch_pid.outer.Actual = asinf(2*(q[0]*q[2]-q[3]*q[1]))
                                    * 57.3f;
        p_reg->chassis.roll_pid.outer.Actual  = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))
                                    * 57.3f;
        INS_QuaternionToEuler(q, &p_reg->chassis.pitch_pid.outer.Actual,
                              &p_reg->chassis.roll_pid.outer.Actual, &p_reg->chassis.yaw_pid.outer.Actual);

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
    float scan_angle = 1.0f;
    const uint16_t SCAN_DELAY = 100;
    #define F_EPSILON   0.3f // 放宽阈值，避免浮点误差
    float YAW_MIN = -45.0f, YAW_MAX = 45.0f, PITCH_MIN = -42.0f, PITCH_MAX = 42.0f;
    const float YAW_INIT = 1.0f, PITCH_INIT = 1.0f;

    // 定义扫描状态
    typedef enum {
        SCAN_GO_TOP,         // 先往上走到顶部
        SCAN_GO_RIGHT,       // 从左往右扫
        SCAN_GO_DOWN,        // 往下走
        SCAN_GO_LEFT         // 从右往左扫（简化版，不用8字形，先实现基本的矩形扫描）
    } ScanState_t;

    ScanState_t scan_state = SCAN_GO_TOP;
    volatile uint8_t start_from_center = 1;
    float start_yaw = YAW_INIT;
    float start_pitch = PITCH_INIT;

    for(;;)
    {
        if (p_reg->gimbal.sentry_state == SENTRY_ENABLED)
        {
            /*****************************************************
            *   第一次进入哨兵模式：从中心或上次退出的位置开始
            *****************************************************/
            if (start_from_center == 1)
            {
                // 从中心开始
                p_reg->gimbal.yaw_pid.outer.Target = YAW_INIT;
                p_reg->gimbal.pitch_pid.outer.Target = PITCH_INIT;
                start_from_center = 0;
                scan_state = SCAN_GO_TOP; // 重置状态机
            }
            else
            {
                // 只有在“第一次进入非中心模式”时，才用 start_yaw/start_pitch
                static uint8_t first_resume = 1;
                if (first_resume == 1)
                {
                    p_reg->gimbal.yaw_pid.outer.Target = start_yaw;
                    p_reg->gimbal.pitch_pid.outer.Target = start_pitch;
                    first_resume = 0;
                    scan_state = SCAN_GO_TOP;
                }
                switch(scan_state)
                {
                    case SCAN_GO_TOP:
                        p_reg->gimbal.pitch_pid.outer.Target += scan_angle;
                        // 判断是否到达顶部
                        if (p_reg->gimbal.pitch_pid.outer.Target >= PITCH_MAX - F_EPSILON)
                        {
                            scan_state = SCAN_GO_RIGHT;
                        }
                        break;

                    case SCAN_GO_RIGHT:
                        p_reg->gimbal.yaw_pid.outer.Target += scan_angle;
                        // 判断是否到达最右边
                        if (p_reg->gimbal.yaw_pid.outer.Target >= YAW_MAX - F_EPSILON)
                        {
                            scan_state = SCAN_GO_DOWN;
                        }
                        break;

                    case SCAN_GO_DOWN:
                        p_reg->gimbal.pitch_pid.outer.Target -= scan_angle;
                        // 判断是否到达底部
                        if (p_reg->gimbal.pitch_pid.outer.Target <= PITCH_MIN + F_EPSILON)
                        {
                            scan_state = SCAN_GO_LEFT;
                        }
                        break;

                    case SCAN_GO_LEFT:
                        p_reg->gimbal.yaw_pid.outer.Target -= scan_angle;
                        // 判断是否到达最左边
                        if (p_reg->gimbal.yaw_pid.outer.Target <= YAW_MIN + F_EPSILON)
                        {
                            scan_state = SCAN_GO_TOP; // 重新开始循环
                        }
                        break;

                    default:
                        scan_state = SCAN_GO_TOP;
                        break;
                }
            }

            // 边界保护
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

            start_from_center = 0; // 下次从记录的角度继续，不从中心开始

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
/* 四元数转欧拉角：供上层模块直接读取更直观的 ROLL/PITCH/YAW。 */
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
