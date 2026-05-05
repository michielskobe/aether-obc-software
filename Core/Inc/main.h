/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RXSM_SODS_Pin GPIO_PIN_0
#define RXSM_SODS_GPIO_Port GPIOB
#define RXSM_SODS_EXTI_IRQn EXTI0_IRQn
#define RXSM_SOE_Pin GPIO_PIN_1
#define RXSM_SOE_GPIO_Port GPIOB
#define RXSM_SOE_EXTI_IRQn EXTI1_IRQn
#define RXSM_LO_Pin GPIO_PIN_2
#define RXSM_LO_GPIO_Port GPIOB
#define RXSM_LO_EXTI_IRQn EXTI2_IRQn
#define PGOOD_Pin GPIO_PIN_12
#define PGOOD_GPIO_Port GPIOB
#define PGOOD_EXTI_IRQn EXTI15_10_IRQn
#define SHDN_Pin GPIO_PIN_13
#define SHDN_GPIO_Port GPIOB
#define PWR_EN_Pin GPIO_PIN_15
#define PWR_EN_GPIO_Port GPIOB
#define NA_Pin GPIO_PIN_7
#define NA_GPIO_Port GPIOC
#define NA_EXTI_IRQn EXTI9_5_IRQn
#define IR_ON_OFF_Pin GPIO_PIN_8
#define IR_ON_OFF_GPIO_Port GPIOC
#define RI_Pin GPIO_PIN_9
#define RI_GPIO_Port GPIOC
#define RI_EXTI_IRQn EXTI9_5_IRQn
#define IR_RX_Pin GPIO_PIN_10
#define IR_RX_GPIO_Port GPIOC
#define IR_TX_Pin GPIO_PIN_11
#define IR_TX_GPIO_Port GPIOC
#define GNSS_RX_Pin GPIO_PIN_12
#define GNSS_RX_GPIO_Port GPIOC
#define GNSS_TX_Pin GPIO_PIN_2
#define GNSS_TX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
