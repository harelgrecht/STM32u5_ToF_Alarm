/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ToF.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for taskToF */
osThreadId_t taskToFHandle;
const osThreadAttr_t taskToF_attributes = {
  .name = "taskToF",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for alarmTask */
osThreadId_t alarmTaskHandle;
const osThreadAttr_t alarmTask_attributes = {
  .name = "alarmTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for alarmQueue */
osMessageQueueId_t alarmQueueHandle;
const osMessageQueueAttr_t alarmQueue_attributes = {
  .name = "alarmQueue"
};
/* Definitions for logQueue */
osMessageQueueId_t logQueueHandle;
const osMessageQueueAttr_t logQueue_attributes = {
  .name = "logQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

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
  /* creation of alarmQueue */
  alarmQueueHandle = osMessageQueueNew (16, sizeof(distanceHandler_t), &alarmQueue_attributes);
  /* creation of logQueue */
  logQueueHandle = osMessageQueueNew (16, sizeof(distanceHandler_t), &logQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of taskToF */
  taskToFHandle = osThreadNew(startToF, NULL, &taskToF_attributes);

  /* creation of alarmTask */
  alarmTaskHandle = osThreadNew(startAlarm, NULL, &alarmTask_attributes);

  /* creation of logTask */
  logTaskHandle = osThreadNew(startLog, NULL, &logTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_startToF */
/**
* @brief Function implementing the taskToF thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startToF */
void startToF(void *argument)
{
  /* USER CODE BEGIN taskToF */
	initToF();

	if (startToFSampling(0x7D, 0x01) != HAL_OK) {
		printf("Failed to start ToF sampling. Halting task.\r\n");
  for(;;) { }
	}
	performDistanceMeasurement();
  /* USER CODE END taskToF */
}

/* USER CODE BEGIN Header_startAlarm */
/**
* @brief Function implementing the alarmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startAlarm */
void startAlarm(void *argument)
{
    /* USER CODE BEGIN alarmTask */
	distanceHandler_t receivedPayload;
	osStatus_t status;
	  printf("task alarm started\n\r");
	/* Infinite loop */
  for(;;) {
	  status = osMessageQueueGet(alarmQueueHandle, &receivedPayload, NULL, osWaitForever);
	  if(status == osOK) {
		  printf("ALARM: Object is too close! Distance: %.2f cm\r\n", receivedPayload.distanceCM);
		  if(receivedPayload.distanceCM < 10.0) {
			  BSP_LED_Toggle(LED_RED);
			  BSP_LED_Toggle(LED_GREEN);
			  BSP_LED_Toggle(LED_BLUE);
		  } else {
			  BSP_LED_Off(LED_RED);
	  	  	  BSP_LED_Off(LED_GREEN);
	  	  	  BSP_LED_Off(LED_BLUE);
		  }
	  }
  }
  /* USER CODE END alarmTask */
}

/* USER CODE BEGIN Header_startLog */
/**
* @brief Function implementing the logTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startLog */
void startLog(void *argument)
{
  /* USER CODE BEGIN logTask */
  /* Infinite loop */
  for(;;) {
  }
  /* USER CODE END logTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

