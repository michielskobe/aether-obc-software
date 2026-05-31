/**
  ******************************************************************************
  * @file           : command_interface.c
  * @brief          : Implementation for command_interface.h
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
#include "command_interface.h"
#include "main.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
typedef void (*CanMessageHandler_t)(const can_rx_msg_t *msg);

typedef struct
{
    uint32_t id;
    CanMessageHandler_t handler;
} CanDispatchEntry_t;

/* Private define ------------------------------------------------------------*/
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* Private macro -------------------------------------------------------------*/
#define ANTENNA_DEPLOYED_FLAG (1U << 14) // 0x4000

/* Private variables ---------------------------------------------------------*/
static const uint8_t CMD_DATA[] = {0x00};
static const uint8_t ACK_DATA[] = {0xFF};

extern osThreadId_t SysOrchestratorHandle; // Declared in main.c, used to set flags from command handlers

/* Private function prototypes -----------------------------------------------*/
static void HandleIfsArmBw1Reply(const can_rx_msg_t *msg);
static void HandleIfsFireBw1Reply(const can_rx_msg_t *msg);
static void HandleIfsArmCgg1Reply(const can_rx_msg_t *msg);

static void HandleCsWakeUp(const can_rx_msg_t *msg);
static void HandleCsCamera1EnableReply(const can_rx_msg_t *msg);
static void HandleCsCamera2EnableReply(const can_rx_msg_t *msg);
static void HandleCsSpiEnableReply(const can_rx_msg_t *msg);

/* Dispatch table ------------------------------------------------------------*/
static const CanDispatchEntry_t dispatch_table[] =
{
    { IFS_ARM_BW1_CAN_REPLY_ID,      HandleIfsArmBw1Reply      },
    { IFS_FIRE_BW1_CAN_REPLY_ID,     HandleIfsFireBw1Reply     },
    { IFS_ARM_CGG1_CAN_REPLY_ID,     HandleIfsArmCgg1Reply     },

    { CS_WAKE_UP_CAN_ID,             HandleCsWakeUp            },
    { CS_CAMERA1_ENABLE_CAN_REPLY_ID, HandleCsCamera1EnableReply },
    { CS_CAMERA2_ENABLE_CAN_REPLY_ID, HandleCsCamera2EnableReply },
    { CS_SPI_ENABLE_CAN_REPLY_ID,     HandleCsSpiEnableReply     },
};

/* Public user code ---------------------------------------------------------*/

void CommandInterface_ProcessMessage(const can_rx_msg_t *msg) {
    for (size_t i = 0; i < ARRAY_SIZE(dispatch_table); i++)
    {
        if (dispatch_table[i].id == msg->RxHeader.StdId)
        {
            dispatch_table[i].handler(msg);
            return;
        }
    }
}

/* Private user code ---------------------------------------------------------*/

static void HandleIfsArmBw1Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Burn wire armed successfully, send FIRE command
        send_can_command(IFS_FIRE_BW1_CAN_ID, CMD_DATA, 1);
    }
    else
    {
        // Retry ARM
        send_can_command(IFS_ARM_BW1_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsFireBw1Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        /* Antenna deployed */
        osThreadFlagsSet(SysOrchestratorHandle, ANTENNA_DEPLOYED_FLAG);
    }
    else
    {
        /* Retry sequence */
        send_can_command(IFS_ARM_BW1_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsArmCgg1Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // CGG1 armed successfully, send FIRE command
        send_can_command(IFS_FIRE_CGG1_CAN_ID, CMD_DATA, 1);
    }
    else
    {
        // Retry ARM
        send_can_command(IFS_ARM_CGG1_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleCsWakeUp(const can_rx_msg_t *msg)
{
    (void)msg;

    // Acknowledge wake-up signal from camera system
    send_can_command(CS_WAKE_UP_CAN_REPLY_ID, ACK_DATA, 1);

    // Instruct camera system to turn on cameras and SPI interface
    send_can_command(CS_CAMERA1_ENABLE_CAN_ID, CMD_DATA, 1);
    send_can_command(CS_CAMERA2_ENABLE_CAN_ID, CMD_DATA, 1);
    send_can_command(CS_SPI_ENABLE_CAN_ID, CMD_DATA, 1);
}

static void HandleCsCamera1EnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Camera 1 system failed to be enabled, retry command
        send_can_command(CS_CAMERA1_ENABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleCsCamera2EnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Camera 2 system failed to be enabled, retry command
        send_can_command(CS_CAMERA2_ENABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleCsSpiEnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Camera SPI interface failed to be enabled, retry command
        send_can_command(CS_SPI_ENABLE_CAN_ID, CMD_DATA, 1);
    }
}