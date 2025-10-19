/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

#include "stm32u5xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32u5xx_nucleo.h"
#include "ToF.h"
#include <stdint.h>
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void initDwt();
void scanI2C(void);
void delay_us(uint32_t microseconds);
void manualI2CInit(void);
HAL_StatusTypeDef i2cRead(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef i2cWrite(uint8_t reg, uint8_t data);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pmod_SS_Pin GPIO_PIN_3
#define pmod_SS_GPIO_Port GPIOF
#define pmod_IRQ_Pin GPIO_PIN_5
#define pmod_IRQ_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
