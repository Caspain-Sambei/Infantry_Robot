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
/* USER CODE END Header_StartUSB_TxTask04 */
void StartUSB_TxTask(void *argument)
{
    /* USER CODE BEGIN StartUSB_TxTask04 */
    SENDPACKET tx_data;
    /* Infinite loop */
    for(;;)
    {
        osStatus_t status = osMessageQueueGet(BMI_USBTxQueueHandle,&tx_data,NULL,osWaitForever);

        if (status == osOK)
        {
            USB_Send(tx_data);
        }
    }
    /* USER CODE END StartUSB_TxTask04 */
}

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
/* USER CODE END Header_StartPIDTask */
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

/* USER CODE END Header_StartexPIDTask */
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
 *								CAN通信
 *				云台接收来自视觉的数据，所以暂时不用接收电机的返回值
 **************************************************************************/
//PID 任务刚读到data1，你还没改完data2/data3/data4，导致 PID 拿到 “半截数据”，云台控制抖 / 失控

void StartCAN_TxTask(void *argument)
{
    /* USER CODE BEGIN StartCAN_Tx */
    /* Infinite loop */
    for(;;)
    {
        // CAN_Send(CAN_6020_1 ,&p_reg->TxData,8);

        osDelay(1);
    }
    /* USER CODE END StartCAN_Tx */
}

/* USER CODE END Header_StartCAN_Rx */
void gimbal_CAN_RxTask(void *argument)
{
    /* USER CODE BEGIN StartCAN_Rx */
    static uint8_t buf[9]  ={0};
    /* Infinite loop */
    for(;;)
    {
        osMessageQueueGet(CAN_1RxQueueHandle,&buf,NULL,osWaitForever);

        if (buf[8] == 1)
        {
            p_reg->gimbal.pitch_RxData.data1 = ((int16_t)buf[1] & 0xFF) | ((int16_t)buf[0] << 8);
            p_reg->gimbal.pitch_RxData.data2 = ((int16_t)buf[3] & 0xFF) | ((int16_t)buf[2] << 8);
            p_reg->gimbal.pitch_RxData.data3 = ((int16_t)buf[5] & 0xFF) | ((int16_t)buf[4] << 8);
            p_reg->gimbal.pitch_RxData.data4 = ((int16_t)buf[7] & 0xFF) | ((int16_t)buf[6] << 8);
        }
        if (buf[8] == 2)
        {
            p_reg->gimbal.yaw_RxData.data1 = ((int16_t)buf[1] &0xFF) | ((int16_t)buf[0]<<8);
            p_reg->gimbal.yaw_RxData.data2 = ((int16_t)buf[3] &0xFF) | ((int16_t)buf[2]<<8);
            p_reg->gimbal.yaw_RxData.data3 = ((int16_t)buf[5] &0xFF) | ((int16_t)buf[4]<<8);
            p_reg->gimbal.yaw_RxData.data4 = ((int16_t)buf[7] &0xFF) | ((int16_t)buf[6]<<8);
        }
        if (buf[8] == 3)
        {
            p_reg->gimbal.pitch_RxData.data1 = ((int16_t)buf[1] & 0xFF) | ((int16_t)buf[0] << 8);
            p_reg->gimbal.pitch_RxData.data2 = ((int16_t)buf[3] & 0xFF) | ((int16_t)buf[2] << 8);
            p_reg->gimbal.pitch_RxData.data3 = ((int16_t)buf[5] & 0xFF) | ((int16_t)buf[4] << 8);
            p_reg->gimbal.pitch_RxData.data4 = ((int16_t)buf[7] & 0xFF) | ((int16_t)buf[6] << 8);
        }
        if (buf[8] == 4)
        {
            p_reg->gimbal.yaw_RxData.data1 = ((int16_t)buf[1] &0xFF) | ((int16_t)buf[0]<<8);
            p_reg->gimbal.yaw_RxData.data2 = ((int16_t)buf[3] &0xFF) | ((int16_t)buf[2]<<8);
            p_reg->gimbal.yaw_RxData.data3 = ((int16_t)buf[5] &0xFF) | ((int16_t)buf[4]<<8);
            p_reg->gimbal.yaw_RxData.data4 = ((int16_t)buf[7] &0xFF) | ((int16_t)buf[6]<<8);
        }
    }
    /* USER CODE END StartCAN_Rx */
}
/***************************************************************************
 *					        BMI088和四元解算
 **************************************************************************/
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

        p_reg->gimbal.curr_angle.yaw  = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
                                    * 57.3f;
        p_reg->gimbal.curr_angle.pitch = asinf(2*(q[0]*q[2]-q[3]*q[1]))
                                    * 57.3f;
        p_reg->gimbal.curr_angle.roll  = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))
                                    * 57.3f;

        osMessageQueuePut(BMI_USBTxQueueHandle,&p_reg->gimbal.curr_angle,0,osWaitForever);
        osDelay(10);
    }
    /* USER CODE END Startbmi088Task */
}

/***************************************************************************
 *			哨兵模式:云台扫描正前方上下左右90°的正方形平面,假定没检测到时视觉不发送消息
 **************************************************************************/
/* USER CODE END Header_StartSentry_modeTask */
void StartSentry_modeTask(void *argument)
{
    /* USER CODE BEGIN StartSentry_modeTask */
    typedef enum {
        SENTRY_SCAN_RESET,    // 初始朝向（右上角45°）
        SENTRY_SCAN_YAW,     // Yaw轴逐度扫描
        SENTRY_SCAN_PITCH,   // Pitch轴逐度调整
        SENTRY_SCAN_IDLE     // 空闲
    } Sentry_Scan_State_t;
    Sentry_Scan_State_t scan_state = SENTRY_SCAN_RESET;

    uint8_t scan_yaw_cnt = 0;  // Yaw扫描计数（0~90）
    uint8_t scan_pitch_cnt = 0;// Pitch扫描计数（0~90）
    const uint32_t SCAN_DELAY = 10; // 扫描步长延时（ms，可调整）

    // 初始角度（右上角45°）
    const float INIT_YAW = 45.0f;
    const float INIT_PITCH = 45.0f;

    /* Infinite loop */
    for(;;)
    {
        // 哨兵模式禁用时才执行扫描
        if (p_reg->gimbal.sentry_state == SENTRY_ENABLED)
        {
            switch(scan_state)
            {
                case SENTRY_SCAN_RESET:
                    p_reg->gimbal.yaw_pid.outer.Target += INIT_YAW;
                    p_reg->gimbal.pitch_pid.outer.Target += INIT_PITCH;

                    p_reg->TxData.data1 = (int16_t)(p_reg->gimbal.yaw_pid.outer.Target); // 放大10倍提高精度
                    p_reg->TxData.data2 = (int16_t)(p_reg->gimbal.pitch_pid.outer.Target);
                        CAN_Send(CAN_6020_1, &p_reg->TxData, 8);
                        memset(&p_reg->TxData, 0, sizeof(CAN_Structure));
                    // 切换到Yaw扫描状态
                    scan_state = SENTRY_SCAN_YAW;
                    scan_yaw_cnt = 0;
                    scan_pitch_cnt = 0;
                    break;

                case SENTRY_SCAN_YAW:
                    // 步骤2：Yaw轴逐度向左扫描（每次任务循环只减1°）
                    if (scan_yaw_cnt < 90)
                    {
                        // 只修改一次角度，避免重复操作
                        p_reg->gimbal.yaw_pid.outer.Target -= 1.0f;
                        // 角度范围保护（-45°~45°）
                        if (p_reg->gimbal.yaw_pid.outer.Target < -45.0f)
                            p_reg->gimbal.yaw_pid.outer.Target = -45.0f;
                        if (p_reg->gimbal.yaw_pid.outer.Target > 45.0f)
                            p_reg->gimbal.yaw_pid.outer.Target = 45.0f;

                        // 更新CAN数据并发送
                        p_reg->TxData.data1 = (int16_t)(p_reg->gimbal.yaw_pid.outer.Target);
                        p_reg->TxData.data2 = (int16_t)(p_reg->gimbal.pitch_pid.outer.Target);
                            CAN_Send(CAN_6020_1, &p_reg->TxData, 8);
                            memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

                        scan_yaw_cnt++;
                    }
                    else
                    {
                        scan_state = SENTRY_SCAN_PITCH;
                        scan_yaw_cnt = 0; // 重置Yaw计数
                    }
                    break;

                case SENTRY_SCAN_PITCH:
                    // 步骤3：Pitch轴逐度向下扫描（每次任务循环只减1°）
                    if (scan_pitch_cnt < 90)
                    {
                        p_reg->gimbal.pitch_pid.outer.Target -= 1.0f;
                        // Pitch角度保护（防止超过机械限位，示例：-30°~60°）
                        // 这里用target还是actual？
                        if (p_reg->gimbal.pitch_pid.outer.Target < -45.0f)
                            p_reg->gimbal.pitch_pid.outer.Target = -45.0f;
                        if (p_reg->gimbal.pitch_pid.outer.Target > 45.0f)
                            p_reg->gimbal.pitch_pid.outer.Target = 45.0f;

                        // 更新CAN数据并发送
                        p_reg->TxData.data1 = (int16_t)(p_reg->gimbal.yaw_pid.outer.Target);
                        p_reg->TxData.data2 = (int16_t)(p_reg->gimbal.pitch_pid.outer.Target);
                            CAN_Send(CAN_6020_1, &p_reg->TxData, 8);
                            memset(&p_reg->TxData, 0, sizeof(CAN_Structure));

                        scan_pitch_cnt++;
                        // 回到Yaw扫描，继续循环
                        scan_state = SENTRY_SCAN_YAW;
                    }
                    else
                    {
                        // 一轮扫描完成，重置状态（重新从初始角度开始）
                        scan_state = SENTRY_SCAN_RESET;
                        scan_pitch_cnt = 0;
                    }
                    break;
            }
        }
        // 核心：释放CPU，让其他任务运行（RTOS必加）
        osDelay(SCAN_DELAY);
    }
    /* USER CODE END StartSentry_modeTask */
}