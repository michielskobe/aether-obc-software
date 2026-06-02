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
#include "can.h"
#include "cmsis_os.h"
#include "command_interface.h"
#include "data_packet.h"
#include "main.h"

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

/* Private variables ---------------------------------------------------------*/
static const uint8_t CMD_DATA[] = {0x00}; // Data payload for commands that don't require specific data, can be used to trigger actions without parameters
static const uint8_t ACK_DATA[] = {0xFF}; // Data payload for acknowledging receipt of a command

extern osThreadId_t SysOrchestratorHandle; // Declared in main.c, used to set flags from command handlers
extern metadata_t mission_metadata; // Declared in main.c

/* Private function prototypes -----------------------------------------------*/
static void HandleEpsPingReply(const can_rx_msg_t *msg);
static void HandleEpsBatteriesEnableReply(const can_rx_msg_t *msg);
static void HandleEpsBatteriesDisableReply(const can_rx_msg_t *msg);
static void HandleEpsChargingEnableReply(const can_rx_msg_t *msg);
static void HandleEpsChargingDisableReply(const can_rx_msg_t *msg);
static void HandleEpsRailEnableReply(const can_rx_msg_t *msg);
static void HandleEpsRailDisableReply(const can_rx_msg_t *msg);
static void HandleEpsPowerCycleReply(const can_rx_msg_t *msg);
static void HandleEpsRadioSilenceAck(const can_rx_msg_t *msg);

static void HandleIfsPingReply(const can_rx_msg_t *msg);
static void HandleIfsArmBw1Reply(const can_rx_msg_t *msg);
static void HandleIfsFireBw1Reply(const can_rx_msg_t *msg);
static void HandleIfsArmBw2Reply(const can_rx_msg_t *msg);
static void HandleIfsFireBw2Reply(const can_rx_msg_t *msg);
static void HandleIfsArmCgg1Reply(const can_rx_msg_t *msg);
static void HandleIfsFireCgg1Reply(const can_rx_msg_t *msg);
static void HandleIfsArmCgg2Reply(const can_rx_msg_t *msg);
static void HandleIfsFireCgg2Reply(const can_rx_msg_t *msg);
static void HandleIfsActuatorResetReply(const can_rx_msg_t *msg);
static void HandleIfsActuatorSpentRequest(const can_rx_msg_t *msg);
static void HandleIfsWakeUp(const can_rx_msg_t *msg);
static void HandleIfsArmDecay(const can_rx_msg_t *msg);
static void HandleIfsFireDecay(const can_rx_msg_t *msg);

static void HandleUhfcomPingReply(const can_rx_msg_t *msg);
static void HandleUhfcomBeaconEnableReply(const can_rx_msg_t *msg);
static void HandleUhfcomBeaconDisableReply(const can_rx_msg_t *msg);
static void HandleUhfcomBeaconDataReply(const can_rx_msg_t *msg);
static void HandleUhfcomWakeUp(const can_rx_msg_t *msg);

static void HandleCsPingReply(const can_rx_msg_t *msg);
static void HandleCsWakeUp(const can_rx_msg_t *msg);
static void HandleCsCamera1EnableReply(const can_rx_msg_t *msg);
static void HandleCsCamera1DisableReply(const can_rx_msg_t *msg);
static void HandleCsCamera2EnableReply(const can_rx_msg_t *msg);
static void HandleCsCamera2DisableReply(const can_rx_msg_t *msg);
static void HandleCsSpiEnableReply(const can_rx_msg_t *msg);
static void HandleCsSpiDisableReply(const can_rx_msg_t *msg);
static void HandleCsPowerCycle(const can_rx_msg_t *msg);

/* Dispatch table ------------------------------------------------------------*/
static const CanDispatchEntry_t dispatch_table[] =
{
    {EPS_PING_CAN_REPLY_ID,                  HandleEpsPingReply},
    {EPS_BATTERIES_ENABLE_CAN_REPLY_ID,      HandleEpsBatteriesEnableReply},
    {EPS_BATTERIES_DISABLE_CAN_REPLY_ID,     HandleEpsBatteriesDisableReply},
    {EPS_CHARGING_ENABLE_CAN_REPLY_ID,       HandleEpsChargingEnableReply},
    {EPS_CHARGING_DISABLE_CAN_REPLY_ID,      HandleEpsChargingDisableReply},
    {EPS_RAIL_ENABLE_CAN_REPLY_ID,           HandleEpsRailEnableReply},
    {EPS_RAIL_DISABLE_CAN_REPLY_ID,          HandleEpsRailDisableReply},
    {EPS_POWER_CYCLE_CAN_REPLY_ID,           HandleEpsPowerCycleReply},
    {EPS_RADIO_SILENCE_ACK_CAN_ID,           HandleEpsRadioSilenceAck},

    {IFS_PING_CAN_REPLY_ID,                 HandleIfsPingReply},
    {IFS_ARM_BW1_CAN_REPLY_ID,              HandleIfsArmBw1Reply},
    {IFS_ARM_BW2_CAN_REPLY_ID,              HandleIfsArmBw2Reply},
    {IFS_ARM_CGG1_CAN_REPLY_ID,             HandleIfsArmCgg1Reply},   
    {IFS_ARM_CGG2_CAN_REPLY_ID,             HandleIfsArmCgg2Reply},
    {IFS_FIRE_BW1_CAN_REPLY_ID,             HandleIfsFireBw1Reply},
    {IFS_FIRE_BW2_CAN_REPLY_ID,             HandleIfsFireBw2Reply},
    {IFS_FIRE_CGG1_CAN_REPLY_ID,            HandleIfsFireCgg1Reply},
    {IFS_FIRE_CGG2_CAN_REPLY_ID,            HandleIfsFireCgg2Reply},
    {IFS_ACTUATOR_RESET_CAN_REPLY_ID,       HandleIfsActuatorResetReply},
    {IFS_WAKE_UP_CAN_ID,                    HandleIfsWakeUp},
    {IFS_ARM_DECAY_CAN_ID,                  HandleIfsArmDecay},
    {IFS_FIRE_DECAY_CAN_ID,                 HandleIfsFireDecay},
    {IFS_REP_ACTUATOR_SPENT_CAN_ID,         HandleIfsActuatorSpentRequest},

    {UHFCOM_PING_CAN_REPLY_ID,              HandleUhfcomPingReply},
    {UHFCOM_BEACON_ENABLE_CAN_REPLY_ID,     HandleUhfcomBeaconEnableReply},
    {UHFCOM_BEACON_DISABLE_CAN_REPLY_ID,    HandleUhfcomBeaconDisableReply},
    {UHFCOM_BEACON_DATA_CAN_REPLY_ID,       HandleUhfcomBeaconDataReply},
    {UHFCOM_WAKE_UP_CAN_ID,                 HandleUhfcomWakeUp},

    {CS_PING_CAN_REPLY_ID,                  HandleCsPingReply},
    {CS_CAMERA1_ENABLE_CAN_REPLY_ID,        HandleCsCamera1EnableReply},
    {CS_CAMERA1_DISABLE_CAN_REPLY_ID,       HandleCsCamera1DisableReply},
    {CS_CAMERA2_ENABLE_CAN_REPLY_ID,        HandleCsCamera2EnableReply},
    {CS_CAMERA2_DISABLE_CAN_REPLY_ID,       HandleCsCamera2DisableReply},
    {CS_SPI_ENABLE_CAN_REPLY_ID,            HandleCsSpiEnableReply},
    {CS_SPI_DISABLE_CAN_REPLY_ID,           HandleCsSpiDisableReply},
    { CS_WAKE_UP_CAN_ID,                    HandleCsWakeUp},
    {CS_POWER_CYCLE_CAN_ID,                 HandleCsPowerCycle}
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

static void HandleEpsPingReply(const can_rx_msg_t *msg)
{
    // Handle EPS ping reply if needed
}

static void HandleEpsBatteriesEnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Batteries enabled successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry command
        send_can_command(EPS_BATTERIES_ENABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleEpsBatteriesDisableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Batteries disabled successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry command
        send_can_command(EPS_BATTERIES_DISABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleEpsChargingEnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Charging enabled successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry command
        send_can_command(EPS_CHARGING_ENABLE_CAN_ID, CMD_DATA, 1);
    }
}
static void HandleEpsChargingDisableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Charging disabled successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry command
        send_can_command(EPS_CHARGING_DISABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleEpsRailEnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Rail enabled successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry command
        send_can_command(EPS_RAIL_ENABLE_CAN_ID, msg->RxData[1], 1);
    }
}

static void HandleEpsRailDisableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Rail disabled successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry command
        send_can_command(EPS_RAIL_DISABLE_CAN_ID, msg->RxData[1], 1);
    }
}

static void HandleEpsPowerCycleReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Power cycle successful
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry command
        send_can_command(EPS_POWER_CYCLE_CAN_ID, msg->RxData[1], 1);
    }
}

static void HandleEpsRadioSilenceAck(const can_rx_msg_t *msg)
{
    // Handle acknowledgment of radio silence
}

static void HandleIfsPingReply(const can_rx_msg_t *msg)
{
    // Handle IFS ping reply if needed
}

static void HandleIfsArmBw1Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Burn wire armed successfully, send FIRE command
        send_can_command(IFS_FIRE_BW1_CAN_ID, CMD_DATA, 1);
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry ARM
        send_can_command(IFS_ARM_BW1_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsFireBw1Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Antenna deployed
        osThreadFlagsSet(SysOrchestratorHandle, ANTENNA_DEPLOYED_FLAG);
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry sequence
        send_can_command(IFS_ARM_BW1_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsArmBw2Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Burn wire armed successfully, send FIRE command
        send_can_command(IFS_FIRE_BW2_CAN_ID, CMD_DATA, 1);
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry ARM
        send_can_command(IFS_ARM_BW2_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsFireBw2Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // Parachute deployed
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry sequence
        send_can_command(IFS_ARM_BW2_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsArmCgg1Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // CGG1 armed successfully, send FIRE command
        send_can_command(IFS_FIRE_CGG1_CAN_ID, CMD_DATA, 1);
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry ARM
        send_can_command(IFS_ARM_CGG1_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsFireCgg1Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // CGG1 fired successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry sequence
        send_can_command(IFS_ARM_CGG1_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsArmCgg2Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // CGG2 armed successfully, send FIRE command
        send_can_command(IFS_FIRE_CGG2_CAN_ID, CMD_DATA, 1);
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry ARM
        send_can_command(IFS_ARM_CGG2_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsFireCgg2Reply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0xFF)
    {
        // CGG2 fired successfully
    }
    else if (msg->RxData[0] == 0x00)
    {
        // Retry sequence
        send_can_command(IFS_ARM_CGG2_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsActuatorResetReply(const can_rx_msg_t *msg)
{
   if (msg->RxData[0] == 0x00)
   {
    // Retry reset command
    send_can_command(IFS_ACTUATOR_RESET_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleIfsActuatorSpentRequest(const can_rx_msg_t *msg)
{
    // Compute a message to send back to the IFS indicating which actuators are spent based on the current state of the actuators array
    uint8_t spent_actuator_mask =
    ((mission_metadata.bw1_fired  != 0) << 0) |
    ((mission_metadata.bw2_fired  != 0) << 1) |
    ((mission_metadata.cgg1_fired != 0) << 2) |
    ((mission_metadata.cgg2_fired != 0) << 3);

    // Send the spent actuator mask back in the reply message so that the IFS can update its internal state accordingly
    send_can_command(IFS_REP_ACTUATOR_SPENT_CAN_REPLY_ID, &spent_actuator_mask, 1);
}

static void HandleIfsWakeUp(const can_rx_msg_t *msg)
{
    (void)msg;

    // Acknowledge wake-up signal from IFS
    send_can_command(IFS_WAKE_UP_CAN_REPLY_ID, ACK_DATA, 1);
}

static void HandleIfsArmDecay(const can_rx_msg_t *msg)
{
    (void)msg;

    // Acknowledge arm decay signal from IFS
    send_can_command(IFS_ARM_DECAY_CAN_REPLY_ID, ACK_DATA, 1);
}

static void HandleIfsFireDecay(const can_rx_msg_t *msg)
{
    (void)msg;

    // Acknowledge fire decay signal from IFS
    send_can_command(IFS_FIRE_DECAY_CAN_REPLY_ID, ACK_DATA, 1);
}

static void HandleUhfcomPingReply(const can_rx_msg_t *msg)
{
    // Handle UHFCOM ping reply if needed
}

static void HandleUhfcomBeaconEnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Send GNSS data to UHFCOM
        send_can_command(UHFCOM_BEACON_DATA_CAN_ID, mission_metadata.gnss, sizeof(mission_metadata.gnss));

        // Retry enabling UHFCOM beacon
        send_can_command(UHFCOM_BEACON_ENABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleUhfcomBeaconDisableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Retry disabling UHFCOM beacon
        send_can_command(UHFCOM_BEACON_DISABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleUhfcomBeaconDataReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Retry sending GNSS data to UHFCOM
        send_can_command(UHFCOM_BEACON_DATA_CAN_ID, mission_metadata.gnss, sizeof(mission_metadata.gnss));
    }

}

static void HandleUhfcomWakeUp(const can_rx_msg_t *msg)
{
    (void)msg;

    // Acknowledge wake-up signal from UHFCOM
    send_can_command(UHFCOM_WAKE_UP_CAN_REPLY_ID, ACK_DATA, 1);
}

static void HandleCsPingReply(const can_rx_msg_t *msg)
{
    // Handle CS ping reply if needed
}

static void HandleCsCamera1EnableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Camera 1 system failed to be enabled, retry command
        send_can_command(CS_CAMERA1_ENABLE_CAN_ID, CMD_DATA, 1);
    }
}

static void HandleCsCamera1DisableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Camera 1 system failed to be disabled, retry command
        send_can_command(CS_CAMERA1_DISABLE_CAN_ID, CMD_DATA, 1);
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

static void HandleCsCamera2DisableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Camera 2 system failed to be disabled, retry command
        send_can_command(CS_CAMERA2_DISABLE_CAN_ID, CMD_DATA, 1);
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

static void HandleCsSpiDisableReply(const can_rx_msg_t *msg)
{
    if (msg->RxData[0] == 0x00)
    {
        // Camera SPI interface failed to be disabled, retry command
        send_can_command(CS_SPI_DISABLE_CAN_ID, CMD_DATA, 1);
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

static void HandleCsPowerCycle(const can_rx_msg_t *msg)
{
    (void)msg;

    // Instruct the EPS to power cycle the camera system 5V rail
    send_can_command(EPS_POWER_CYCLE_CAN_ID, (uint8_t[]){CS_5V_RAIL_ID}, 1);
}