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
#include "cmsis_os.h"
#include <stdbool.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t cmd_id;        /**< CAN ID that was transmitted              */
    uint32_t reply_id;      /**< CAN ID expected in reply                 */
    uint8_t  data[8];       /**< Copy of the payload                      */
    uint8_t  data_len;      /**< Payload length                           */
    uint32_t sent_at_tick;  /**< osKernelGetTickCount() at last send      */
    bool     active;        /**< Slot is currently in use                 */
} PendingCmd_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1; // CAN handle declared in main.c

static PendingCmd_t pending_cmds[PENDING_CMD_MAX]; // Array to track pending commands that have been sent but not yet acknowledged
static osMutexId_t pending_cmd_mutex;

const osMutexAttr_t Pending_Cmd_Mutex_attr = {
  "PendingCmdMutex",                          // human readable mutex name
  osMutexPrioInherit | osMutexRobust,    // attr_bits
  NULL,                                     // memory for control block   
  0U                                        // size for control block
};

/* Private function prototypes -----------------------------------------------*/
static void pending_cmd_register(uint16_t cmd_id, uint16_t reply_id, const uint8_t *data, uint8_t len);

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

void send_can_command_tracked(uint16_t cmd_id, uint16_t reply_id, const uint8_t *data, uint8_t dlc)
{
    /* Transmit before registering so the slot is live immediately */
    send_can_command(cmd_id, data, dlc);
    osMutexAcquire(pending_cmd_mutex, osWaitForever);
    pending_cmd_register(cmd_id, reply_id, data, dlc);
    osMutexRelease(pending_cmd_mutex);
}

void can_pending_init(void)
{
    memset(pending_cmds, 0, sizeof(pending_cmds));
    pending_cmd_mutex = osMutexNew(&Pending_Cmd_Mutex_attr);
}

void can_pending_clear(uint16_t reply_id)
{
    osMutexAcquire(pending_cmd_mutex, osWaitForever);
    for (int i = 0; i < PENDING_CMD_MAX; i++)
    {
        if (pending_cmds[i].active && pending_cmds[i].reply_id == reply_id)
        {
            pending_cmds[i].active = false;
            return;
        }
    }
    osMutexRelease(pending_cmd_mutex);
}

void can_pending_retry(void)
{
    uint32_t now = osKernelGetTickCount();
    
    osMutexAcquire(pending_cmd_mutex, osWaitForever);
    for (int i = 0; i < PENDING_CMD_MAX; i++)
    {
        PendingCmd_t *p = &pending_cmds[i];
 
        if (!p->active)
        { 
            continue; 
        }

        if ((now - p->sent_at_tick) < pdMS_TO_TICKS(PENDING_CMD_TIMEOUT_MS))
        { 
            continue; 
        }
 
        // Timeout has occurred for this pending command, attempt retry
        p->sent_at_tick = now;
        send_can_command(p->cmd_id, p->data, p->data_len);
    }
    osMutexRelease(pending_cmd_mutex);
}

static void pending_cmd_register(uint16_t cmd_id, uint16_t reply_id, const uint8_t *data, uint8_t len)
{
    for (int i = 0; i < PENDING_CMD_MAX; i++)
    {
        if (!pending_cmds[i].active)
        {
            pending_cmds[i].cmd_id     = cmd_id;
            pending_cmds[i].reply_id   = reply_id;
            pending_cmds[i].data_len   = len;
            pending_cmds[i].sent_at_tick = osKernelGetTickCount();
            pending_cmds[i].active     = true;
            memcpy(pending_cmds[i].data, data, len);
            return;
        }
    }
    /* Table full: command was already sent but won't be retried.
       Increase PENDING_CMD_MAX if this occurs in practice. */
}

