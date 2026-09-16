/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
/* Definitions for StarttTask */
osThreadId_t StarttTaskHandle;
const osThreadAttr_t StarttTask_attributes = {
  .name = "StarttTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RcDecodeTask */
osThreadId_t RcDecodeTaskHandle;
const osThreadAttr_t RcDecodeTask_attributes = {
  .name = "RcDecodeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ImuDecodeTask */
osThreadId_t ImuDecodeTaskHandle;
const osThreadAttr_t ImuDecodeTask_attributes = {
  .name = "ImuDecodeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for PantilelTask */
osThreadId_t PantilelTaskHandle;
const osThreadAttr_t PantilelTask_attributes = {
  .name = "PantilelTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ShooterTask */
osThreadId_t ShooterTaskHandle;
const osThreadAttr_t ShooterTask_attributes = {
  .name = "ShooterTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RcQuque */
osMessageQueueId_t RcQuqueHandle;
const osMessageQueueAttr_t RcQuque_attributes = {
  .name = "RcQuque"
};
/* Definitions for JudementQueue */
osMessageQueueId_t JudementQueueHandle;
const osMessageQueueAttr_t JudementQueue_attributes = {
  .name = "JudementQueue"
};
/* Definitions for ImuQueue */
osMessageQueueId_t ImuQueueHandle;
const osMessageQueueAttr_t ImuQueue_attributes = {
  .name = "ImuQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTaskFuction(void *argument);
void StartRcDecode(void *argument);
void StartImuDecode(void *argument);
void PantileControlTask(void *argument);
void ChassisControlTask(void *argument);
void ShooterControlTask(void *argument);

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
  /* creation of RcQuque */
  RcQuqueHandle = osMessageQueueNew (100, sizeof(uint32_t), &RcQuque_attributes);

  /* creation of JudementQueue */
  JudementQueueHandle = osMessageQueueNew (100, sizeof(uint16_t), &JudementQueue_attributes);

  /* creation of ImuQueue */
  ImuQueueHandle = osMessageQueueNew (100, sizeof(uint32_t), &ImuQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StarttTask */
  StarttTaskHandle = osThreadNew(StartTaskFuction, NULL, &StarttTask_attributes);

  /* creation of RcDecodeTask */
  RcDecodeTaskHandle = osThreadNew(StartRcDecode, NULL, &RcDecodeTask_attributes);

  /* creation of ImuDecodeTask */
  ImuDecodeTaskHandle = osThreadNew(StartImuDecode, NULL, &ImuDecodeTask_attributes);

  /* creation of PantilelTask */
  PantilelTaskHandle = osThreadNew(PantileControlTask, NULL, &PantilelTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(ChassisControlTask, NULL, &ChassisTask_attributes);

  /* creation of ShooterTask */
  ShooterTaskHandle = osThreadNew(ShooterControlTask, NULL, &ShooterTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskFuction */
/**
  * @brief  Function implementing the StarttTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskFuction */
void StartTaskFuction(void *argument)
{
  /* USER CODE BEGIN StartTaskFuction */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskFuction */
}

/* USER CODE BEGIN Header_StartRcDecode */
/**
* @brief Function implementing the RcDecodeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRcDecode */
void StartRcDecode(void *argument)
{
  /* USER CODE BEGIN StartRcDecode */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRcDecode */
}

/* USER CODE BEGIN Header_StartImuDecode */
/**
* @brief Function implementing the ImuDecodeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuDecode */
void StartImuDecode(void *argument)
{
  /* USER CODE BEGIN StartImuDecode */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartImuDecode */
}

/* USER CODE BEGIN Header_PantileControlTask */
/**
* @brief Function implementing the PantilelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PantileControlTask */
void PantileControlTask(void *argument)
{
  /* USER CODE BEGIN PantileControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PantileControlTask */
}

/* USER CODE BEGIN Header_ChassisControlTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisControlTask */
void ChassisControlTask(void *argument)
{
  /* USER CODE BEGIN ChassisControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisControlTask */
}

/* USER CODE BEGIN Header_ShooterControlTask */
/**
* @brief Function implementing the ShooterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ShooterControlTask */
void ShooterControlTask(void *argument)
{
  /* USER CODE BEGIN ShooterControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ShooterControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

