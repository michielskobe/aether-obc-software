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
#define RMU_STARTUP_FLAG      (1U << 0)  // 0x0001
#define GNSS_STARTUP_FLAG     (1U << 1)  // 0x0002
#define IRIDIUM_STARTUP_FLAG  (1U << 2)  // 0x0004
#define SD_CARD_INIT_FLAG     (1U << 3)  // 0x0008
#define SD_CARD_STARTUP_FLAG  (1U << 4)  // 0x0010
#define DATA_ACQ_STARTUP_FLAG (1U << 5)  // 0x0020
#define LO_FLAG               (1U << 6)  // 0x0040
#define SODS_FLAG             (1U << 7)  // 0x0080
#define SOE_FLAG              (1U << 8)  // 0x0100
#define FFU_EJECTION_FLAG     (1U << 9)  // 0x0200
#define RMU_SHUTDOWN_FLAG     (1U << 10) // 0x0400
#define IFS_WAKEUP_FLAG       (1U << 11) // 0x0800
#define UHFCOM_WAKEUP_FLAG    (1U << 12) // 0x1000
#define CS_WAKEUP_FLAG        (1U << 13) // 0x2000
#define ANTENNA_DEPLOYED_FLAG (1U << 14) // 0x4000

#define IFS_3V3_RAIL_ID 0x01
#define IFS_5V_RAIL_ID 0x02
#define GNSS_3V3_RAIL_ID 0x04
#define IRIDIUM_5V_RAIL_ID 0x08
#define CS_5V_RAIL_ID 0x10
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
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
