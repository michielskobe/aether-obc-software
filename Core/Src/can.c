/**
  ******************************************************************************
  * @file           : can.c
  * @brief          : Implementation for can.h
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1; // CAN handle declared in main.c

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

void send_can_command(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {
        .IDE   = CAN_ID_STD,
        .StdId = id,
        .RTR   = CAN_RTR_DATA,
        .DLC   = dlc
    };

    uint32_t mailbox;

    if (HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox) != HAL_OK)
    {
        Error_Handler();
    }
}