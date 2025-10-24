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
distanceHandler_t globalPayload;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern RTC_HandleTypeDef hrtc;
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
/* Definitions for alarmMutex */
osMutexId_t alarmMutexHandle;
const osMutexAttr_t alarmMutex_attributes = {
  .name = "alarmMutex"
};
/* Definitions for alarmQueue */
osMessageQueueId_t alarmQueueHandle;
const osMessageQueueAttr_t alarmQueue_attributes = {
  .name = "alarmQueue"
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
  /* creation of alarmMutex */
  alarmMutexHandle = osMutexNew(&alarmMutex_attributes);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of taskToF */
  taskToFHandle = osThreadNew(startToF, NULL, &taskToF_attributes);

  /* creation of alarmTask */
  alarmTaskHandle = osThreadNew(startAlarm, NULL, &alarmTask_attributes);

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
		for(;;)
			osDelay(osWaitForever);
	}

//	distanceHandler_t localPayload;
	MovingAverageFilter_t distanceFilter;
	initMovingAverage(&distanceFilter);
	double distanceMeters;

	/* Infinite loop */
	for(;;) {
		performToFCalibration();
		distanceMeters = readToFDistance();
		if (distanceMeters >= 0) {
			if(osMutexAcquire(alarmMutexHandle, 10) == osOK) {
				globalPayload.upTimeStamp = HAL_GetTick();
				globalPayload.distanceCM = updateMovingAverage(&distanceFilter, distanceMeters * 100.0);
				osMutexRelease(alarmMutexHandle);
			} else {
				printf("Measurement failed.\r\n");
			}
		}
		osDelay(100);
	}
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
	printf("task alarm started\n\r");
	distanceHandler_t alarmTaskPayload;
	uint32_t hours, minutes, seconds;
	int slowBlinkCnt = 0;
	/* Infinite loop */
	for(;;) {
		if(osMutexAcquire(alarmMutexHandle, 10) == osOK) {
			alarmTaskPayload = globalPayload;
			osMutexRelease(alarmMutexHandle);

			calcUptime(alarmTaskPayload.upTimeStamp, &hours, &minutes, &seconds);
			printf("Distance: %.2f cm |  Uptime: %02lu:%02lu:%02lu\n\r", alarmTaskPayload.distanceCM, hours, minutes, seconds);
			if(alarmTaskPayload.distanceCM < 10.0) {
				printf("ALARM: Object is too close!\n\r");
				blinkLed();
				slowBlinkCnt = 0;
			} else if(alarmTaskPayload.distanceCM < 30.0) {
				slowBlinkCnt++;
				if(slowBlinkCnt >= 10) {
					BSP_LED_Toggle(LED_BLUE);
					slowBlinkCnt = 0;
				}
				BSP_LED_Off(LED_RED);
			} else {
				BSP_LED_Off(LED_RED);
				BSP_LED_Off(LED_GREEN);
				BSP_LED_Off(LED_BLUE);
				slowBlinkCnt = 0;
			}
		}
		osDelay(50);
	}
  /* USER CODE END alarmTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void blinkLed() {
	BSP_LED_On(LED_RED);
	BSP_LED_On(LED_GREEN);
	BSP_LED_On(LED_BLUE);
	osDelay(500); // ms
	BSP_LED_Off(LED_RED);
	BSP_LED_Off(LED_GREEN);
	BSP_LED_Off(LED_BLUE);
	osDelay(500);
}

void calcUptime(uint32_t ms, uint32_t *hh, uint32_t *mm, uint32_t *ss) {
	uint32_t totalSeconds = ms / 1000;
	*hh = totalSeconds / 3600;
	*mm = (totalSeconds % 3600) / 60;
	*ss = totalSeconds % 60;
}
/* USER CODE END Application */

