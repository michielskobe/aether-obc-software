/**
  ******************************************************************************
  * @file           : mosaic_x5.h
  * @brief          : Header for mosaic_x5.c file.
  *                   This file contains the defines for the mosaic-X5 GNSS receiver functionality.
  * @author         : Kobe Michiels
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOSAIC_X5_H
#define __MOSAIC_X5_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define GNSS_DATA_AVAILABLE      (1U << 0)  // 0x0001

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Initialize the mosaic-X5 GNSS receiver. */
int mosaic_x5_init(void);

/* FreeRTOS Task Entry Point Handler */
void gnss_data_handler(void);

/* Callback function for UART receive interrupt. */
void mosaic_uart_rx_cb(uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __MOSAIC_X5_H */