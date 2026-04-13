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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for bmi088Task */
osThreadId_t bmi088TaskHandle;
const osThreadAttr_t bmi088Task_attributes = {
  .name = "bmi088Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for USB_RxTask */
osThreadId_t USB_RxTaskHandle;
const osThreadAttr_t USB_RxTask_attributes = {
  .name = "USB_RxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for chassis_inPID */
osThreadId_t chassis_inPIDHandle;
const osThreadAttr_t chassis_inPID_attributes = {
  .name = "chassis_inPID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for sentry_mode */
osThreadId_t sentry_modeHandle;
const osThreadAttr_t sentry_mode_attributes = {
  .name = "sentry_mode",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RC_Task */
osThreadId_t RC_TaskHandle;
const osThreadAttr_t RC_Task_attributes = {
  .name = "RC_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for USBRxQueue */
osMessageQueueId_t USBRxQueueHandle;
const osMessageQueueAttr_t USBRxQueue_attributes = {
  .name = "USBRxQueue"
};
/* Definitions for RCQueue */
osMessageQueueId_t RCQueueHandle;
const osMessageQueueAttr_t RCQueue_attributes = {
  .name = "RCQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void gimbal_inPIDTask(void *argument);
void Startbmi088Task(void *argument);
void StartUSB_RxTask(void *argument);
void chassis_inPIDTask(void *argument);
void StartSentry_modeTask(void *argument);
void StartRC_Task(void *argument);

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

  /* creation of RCQueue */
  RCQueueHandle = osMessageQueueNew (16, 18, &RCQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of gimbal_inPID */
  gimbal_inPIDHandle = osThreadNew(gimbal_inPIDTask, NULL, &gimbal_inPID_attributes);

  /* creation of bmi088Task */
  bmi088TaskHandle = osThreadNew(Startbmi088Task, NULL, &bmi088Task_attributes);

  /* creation of USB_RxTask */
  USB_RxTaskHandle = osThreadNew(StartUSB_RxTask, NULL, &USB_RxTask_attributes);

  /* creation of chassis_inPID */
  chassis_inPIDHandle = osThreadNew(chassis_inPIDTask, NULL, &chassis_inPID_attributes);

  /* creation of sentry_mode */
  sentry_modeHandle = osThreadNew(StartSentry_modeTask, NULL, &sentry_mode_attributes);

  /* creation of RC_Task */
  RC_TaskHandle = osThreadNew(StartRC_Task, NULL, &RC_Task_attributes);

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

/* USER CODE BEGIN Header_StartSentry_modeTask */
/**
* @brief Function implementing the sentry_mode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSentry_modeTask */
__weak void StartSentry_modeTask(void *argument)
{
  /* USER CODE BEGIN StartSentry_modeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSentry_modeTask */
}

/* USER CODE BEGIN Header_StartRC_Task */
/**
* @brief Function implementing the RC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRC_Task */
__weak void StartRC_Task(void *argument)
{
  /* USER CODE BEGIN StartRC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRC_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

