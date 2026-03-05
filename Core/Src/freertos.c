/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for gimbal_inPID */
osThreadId_t gimbal_inPIDHandle;
const osThreadAttr_t gimbal_inPID_attributes = {
  .name = "gimbal_inPID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for bmi088Task */
osThreadId_t bmi088TaskHandle;
const osThreadAttr_t bmi088Task_attributes = {
  .name = "bmi088Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for USB_TxTask */
osThreadId_t USB_TxTaskHandle;
const osThreadAttr_t USB_TxTask_attributes = {
  .name = "USB_TxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for USB_RxTask */
osThreadId_t USB_RxTaskHandle;
const osThreadAttr_t USB_RxTask_attributes = {
  .name = "USB_RxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for CAN_TxTask */
osThreadId_t CAN_TxTaskHandle;
const osThreadAttr_t CAN_TxTask_attributes = {
  .name = "CAN_TxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for gimbal_CANTask */
osThreadId_t gimbal_CANTaskHandle;
const osThreadAttr_t gimbal_CANTask_attributes = {
  .name = "gimbal_CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for gimbal_exPID */
osThreadId_t gimbal_exPIDHandle;
const osThreadAttr_t gimbal_exPID_attributes = {
  .name = "gimbal_exPID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SbusTransTask */
osThreadId_t SbusTransTaskHandle;
const osThreadAttr_t SbusTransTask_attributes = {
  .name = "SbusTransTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for SubsProcessTask */
osThreadId_t SubsProcessTaskHandle;
const osThreadAttr_t SubsProcessTask_attributes = {
  .name = "SubsProcessTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for chassis_CANTask */
osThreadId_t chassis_CANTaskHandle;
const osThreadAttr_t chassis_CANTask_attributes = {
  .name = "chassis_CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for chassis_calcul */
osThreadId_t chassis_calculHandle;
const osThreadAttr_t chassis_calcul_attributes = {
  .name = "chassis_calcul",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for chassis_inPID */
osThreadId_t chassis_inPIDHandle;
const osThreadAttr_t chassis_inPID_attributes = {
  .name = "chassis_inPID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for chassis_exPID */
osThreadId_t chassis_exPIDHandle;
const osThreadAttr_t chassis_exPID_attributes = {
  .name = "chassis_exPID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for USBRxQueue */
osMessageQueueId_t USBRxQueueHandle;
const osMessageQueueAttr_t USBRxQueue_attributes = {
  .name = "USBRxQueue"
};
/* Definitions for USBTxQueue */
osMessageQueueId_t USBTxQueueHandle;
const osMessageQueueAttr_t USBTxQueue_attributes = {
  .name = "USBTxQueue"
};
/* Definitions for CAN_1RxQueue */
osMessageQueueId_t CAN_1RxQueueHandle;
const osMessageQueueAttr_t CAN_1RxQueue_attributes = {
  .name = "CAN_1RxQueue"
};
/* Definitions for BMI_USBTxQueue */
osMessageQueueId_t BMI_USBTxQueueHandle;
const osMessageQueueAttr_t BMI_USBTxQueue_attributes = {
  .name = "BMI_USBTxQueue"
};
/* Definitions for SbusFrameQueue */
osMessageQueueId_t SbusFrameQueueHandle;
const osMessageQueueAttr_t SbusFrameQueue_attributes = {
  .name = "SbusFrameQueue"
};
/* Definitions for Sbus_2ndQueue */
osMessageQueueId_t Sbus_2ndQueueHandle;
const osMessageQueueAttr_t Sbus_2ndQueue_attributes = {
  .name = "Sbus_2ndQueue"
};
/* Definitions for CAN_2RxQueue */
osMessageQueueId_t CAN_2RxQueueHandle;
const osMessageQueueAttr_t CAN_2RxQueue_attributes = {
  .name = "CAN_2RxQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void gimbal_inPIDTask(void *argument);
void Startbmi088Task(void *argument);
void StartUSB_TxTask(void *argument);
void StartUSB_RxTask(void *argument);
void StartCAN_TxTask(void *argument);
void gimbal_CAN_RxTask(void *argument);
void gimbal_exPIDTask(void *argument);
void StartSbusTransTask(void *argument);
void StartSubsProcessTask(void *argument);
void chassis_CAN_RxTask(void *argument);
void chassis_calculateTask(void *argument);
void chassis_inPIDTask(void *argument);
void chassis_exPIDTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of USBRxQueue */
  USBRxQueueHandle = osMessageQueueNew (16, 10, &USBRxQueue_attributes);

  /* creation of USBTxQueue */
  USBTxQueueHandle = osMessageQueueNew (16, 14, &USBTxQueue_attributes);

  /* creation of CAN_1RxQueue */
  CAN_1RxQueueHandle = osMessageQueueNew (16, 9, &CAN_1RxQueue_attributes);

  /* creation of BMI_USBTxQueue */
  BMI_USBTxQueueHandle = osMessageQueueNew (16, 12, &BMI_USBTxQueue_attributes);

  /* creation of SbusFrameQueue */
  SbusFrameQueueHandle = osMessageQueueNew (16, 18, &SbusFrameQueue_attributes);

  /* creation of Sbus_2ndQueue */
  Sbus_2ndQueueHandle = osMessageQueueNew (16, 20, &Sbus_2ndQueue_attributes);

  /* creation of CAN_2RxQueue */
  CAN_2RxQueueHandle = osMessageQueueNew (16, 9, &CAN_2RxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of gimbal_inPID */
  gimbal_inPIDHandle = osThreadNew(gimbal_inPIDTask, NULL, &gimbal_inPID_attributes);

  /* creation of bmi088Task */
  bmi088TaskHandle = osThreadNew(Startbmi088Task, NULL, &bmi088Task_attributes);

  /* creation of USB_TxTask */
  USB_TxTaskHandle = osThreadNew(StartUSB_TxTask, NULL, &USB_TxTask_attributes);

  /* creation of USB_RxTask */
  USB_RxTaskHandle = osThreadNew(StartUSB_RxTask, NULL, &USB_RxTask_attributes);

  /* creation of CAN_TxTask */
  CAN_TxTaskHandle = osThreadNew(StartCAN_TxTask, NULL, &CAN_TxTask_attributes);

  /* creation of gimbal_CANTask */
  gimbal_CANTaskHandle = osThreadNew(gimbal_CAN_RxTask, NULL, &gimbal_CANTask_attributes);

  /* creation of gimbal_exPID */
  gimbal_exPIDHandle = osThreadNew(gimbal_exPIDTask, NULL, &gimbal_exPID_attributes);

  /* creation of SbusTransTask */
  SbusTransTaskHandle = osThreadNew(StartSbusTransTask, NULL, &SbusTransTask_attributes);

  /* creation of SubsProcessTask */
  SubsProcessTaskHandle = osThreadNew(StartSubsProcessTask, NULL, &SubsProcessTask_attributes);

  /* creation of chassis_CANTask */
  chassis_CANTaskHandle = osThreadNew(chassis_CAN_RxTask, NULL, &chassis_CANTask_attributes);

  /* creation of chassis_calcul */
  chassis_calculHandle = osThreadNew(chassis_calculateTask, NULL, &chassis_calcul_attributes);

  /* creation of chassis_inPID */
  chassis_inPIDHandle = osThreadNew(chassis_inPIDTask, NULL, &chassis_inPID_attributes);

  /* creation of chassis_exPID */
  chassis_exPIDHandle = osThreadNew(chassis_exPIDTask, NULL, &chassis_exPID_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_gimbal_inPIDTask */
/**
  * @brief  Function implementing the gimbal_inPID thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_gimbal_inPIDTask */
__weak void gimbal_inPIDTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN gimbal_inPIDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_inPIDTask */
}

/* USER CODE BEGIN Header_Startbmi088Task */
/**
* @brief Function implementing the bmi088Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startbmi088Task */
__weak void Startbmi088Task(void *argument)
{
  /* USER CODE BEGIN Startbmi088Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Startbmi088Task */
}

/* USER CODE BEGIN Header_StartUSB_TxTask */
/**
* @brief Function implementing the USB_TxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSB_TxTask */
__weak void StartUSB_TxTask(void *argument)
{
  /* USER CODE BEGIN StartUSB_TxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUSB_TxTask */
}

/* USER CODE BEGIN Header_StartUSB_RxTask */
/**
* @brief Function implementing the USB_RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSB_RxTask */
__weak void StartUSB_RxTask(void *argument)
{
  /* USER CODE BEGIN StartUSB_RxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUSB_RxTask */
}

/* USER CODE BEGIN Header_StartCAN_TxTask */
/**
* @brief Function implementing the CAN_TxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN_TxTask */
__weak void StartCAN_TxTask(void *argument)
{
  /* USER CODE BEGIN StartCAN_TxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCAN_TxTask */
}

/* USER CODE BEGIN Header_gimbal_CAN_RxTask */
/**
* @brief Function implementing the gimbal_CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_CAN_RxTask */
__weak void gimbal_CAN_RxTask(void *argument)
{
  /* USER CODE BEGIN gimbal_CAN_RxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_CAN_RxTask */
}

/* USER CODE BEGIN Header_gimbal_exPIDTask */
/**
* @brief Function implementing the gimbal_exPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_exPIDTask */
__weak void gimbal_exPIDTask(void *argument)
{
  /* USER CODE BEGIN gimbal_exPIDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_exPIDTask */
}

/* USER CODE BEGIN Header_StartSbusTransTask */
/**
* @brief Function implementing the SbusTransTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSbusTransTask */
__weak void StartSbusTransTask(void *argument)
{
  /* USER CODE BEGIN StartSbusTransTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSbusTransTask */
}

/* USER CODE BEGIN Header_StartSubsProcessTask */
/**
* @brief Function implementing the SubsProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSubsProcessTask */
__weak void StartSubsProcessTask(void *argument)
{
  /* USER CODE BEGIN StartSubsProcessTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSubsProcessTask */
}

/* USER CODE BEGIN Header_chassis_CAN_RxTask */
/**
* @brief Function implementing the chassis_CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_CAN_RxTask */
__weak void chassis_CAN_RxTask(void *argument)
{
  /* USER CODE BEGIN chassis_CAN_RxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_CAN_RxTask */
}

/* USER CODE BEGIN Header_chassis_calculateTask */
/**
* @brief Function implementing the chassis_calcul thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_calculateTask */
__weak void chassis_calculateTask(void *argument)
{
  /* USER CODE BEGIN chassis_calculateTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_calculateTask */
}

/* USER CODE BEGIN Header_chassis_inPIDTask */
/**
* @brief Function implementing the chassis_inPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_inPIDTask */
__weak void chassis_inPIDTask(void *argument)
{
  /* USER CODE BEGIN chassis_inPIDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_inPIDTask */
}

/* USER CODE BEGIN Header_chassis_exPIDTask */
/**
* @brief Function implementing the chassis_exPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_exPIDTask */
__weak void chassis_exPIDTask(void *argument)
{
  /* USER CODE BEGIN chassis_exPIDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_exPIDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

