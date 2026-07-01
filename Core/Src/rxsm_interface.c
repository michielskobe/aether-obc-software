/**
  ******************************************************************************
  * @file           : rxsm_interface.c
  * @brief          : Implementation for rxsm_interface.h
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
#include "rxsm_interface.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32l4xx_hal_uart.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef void (*RXSMMessageHandler_t)(const RXSM_Telecommand_t *tc);

typedef struct
{
    uint32_t id;
    RXSMMessageHandler_t handler;
} RXSMDispatchEntry_t;

typedef enum
{
    RX_WAIT_SYNC1,
    RX_WAIT_SYNC2,
    RX_WAIT_CMD_H,
    RX_WAIT_CMD_L,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CRC_H,
    RX_WAIT_CRC_L
} rxsm_state_t;

/* Private define ------------------------------------------------------------*/
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t rxsm_rx_buffer[RXSM_RX_BUFFER_SIZE];
static uint8_t rxsm_tx_frame[RXSM_TC_MAX_PACKET];
static volatile uint16_t rxsm_rx_head;
static volatile uint16_t rxsm_rx_tail;

static struct
{
    rxsm_state_t state;
    uint16_t cmd_id;
    uint8_t len;
    uint8_t index;
    uint8_t payload[RXSM_TC_MAX_PAYLOAD];
    uint16_t crc;
} parser;

extern CRC_HandleTypeDef hcrc; // Declared in main.c
extern UART_HandleTypeDef hlpuart1; // Declared in main.c
extern osThreadId_t SysOrchestratorHandle; // Declared in main.c
extern mission_mode_t mission_mode; // Declared in main.c

/* Private function prototypes -----------------------------------------------*/
static void HandleEpsPing(const RXSM_Telecommand_t *tc);
static void HandleEpsBatteriesEnable(const RXSM_Telecommand_t *tc);
static void HandleEpsBatteriesDisable(const RXSM_Telecommand_t *tc);
static void HandleEpsChargingEnable(const RXSM_Telecommand_t *tc);
static void HandleEpsChargingDisable(const RXSM_Telecommand_t *tc);
static void HandleEpsRailEnable(const RXSM_Telecommand_t *tc);
static void HandleEpsRailDisable(const RXSM_Telecommand_t *tc);
static void HandleEpsPowerCycle(const RXSM_Telecommand_t *tc);

static void HandleIfsPing(const RXSM_Telecommand_t *tc);
static void HandleIfsArmBw1(const RXSM_Telecommand_t *tc);
static void HandleIfsArmBw2(const RXSM_Telecommand_t *tc);
static void HandleIfsArmCgg1(const RXSM_Telecommand_t *tc);
static void HandleIfsArmCgg2(const RXSM_Telecommand_t *tc);
static void HandleIfsFireBw1(const RXSM_Telecommand_t *tc);
static void HandleIfsFireBw2(const RXSM_Telecommand_t *tc);
static void HandleIfsFireCgg1(const RXSM_Telecommand_t *tc);
static void HandleIfsFireCgg2(const RXSM_Telecommand_t *tc);
static void HandleIfsActuatorReset(const RXSM_Telecommand_t *tc);

static void HandleUhfcomPing(const RXSM_Telecommand_t *tc);
static void HandleUhfcomBeaconEnable(const RXSM_Telecommand_t *tc);
static void HandleUhfcomBeaconDisable(const RXSM_Telecommand_t *tc);
static void HandleUhfcomBeaconData(const RXSM_Telecommand_t *tc);

static void HandleCsPing(const RXSM_Telecommand_t *tc);
static void HandleCsCamera1Enable(const RXSM_Telecommand_t *tc);
static void HandleCsCamera1Disable(const RXSM_Telecommand_t *tc);
static void HandleCsCamera2Enable(const RXSM_Telecommand_t *tc);
static void HandleCsCamera2Disable(const RXSM_Telecommand_t *tc);
static void HandleCsSpiEnable(const RXSM_Telecommand_t *tc);
static void HandleCsSpiDisable(const RXSM_Telecommand_t *tc);

static void HandleSetTestMode(const RXSM_Telecommand_t *tc);
static void HandleSetFlightMode(const RXSM_Telecommand_t *tc);
static void HandleSimulateLO(const RXSM_Telecommand_t *tc);
static void HandleSimulateSODS(const RXSM_Telecommand_t *tc);
static void HandleSimulateSOE(const RXSM_Telecommand_t *tc);
static void HandleSimulateEjection(const RXSM_Telecommand_t *tc);

/* Dispatch table ------------------------------------------------------------*/
static const RXSMDispatchEntry_t dispatch_table[] =
{
    {EPS_PING_RXSM_ID,                  HandleEpsPing},
    {EPS_BATTERIES_ENABLE_RXSM_ID,      HandleEpsBatteriesEnable},
    {EPS_BATTERIES_DISABLE_RXSM_ID,     HandleEpsBatteriesDisable},
    {EPS_CHARGING_ENABLE_RXSM_ID,       HandleEpsChargingEnable},
    {EPS_CHARGING_DISABLE_RXSM_ID,      HandleEpsChargingDisable},
    {EPS_RAIL_ENABLE_RXSM_ID,           HandleEpsRailEnable},
    {EPS_RAIL_DISABLE_RXSM_ID,          HandleEpsRailDisable},
    {EPS_POWER_CYCLE_RXSM_ID,           HandleEpsPowerCycle},

    {IFS_PING_RXSM_ID,                 HandleIfsPing},
    {IFS_ARM_BW1_RXSM_ID,              HandleIfsArmBw1},
    {IFS_ARM_BW2_RXSM_ID,             HandleIfsArmBw2},
    {IFS_ARM_CGG1_RXSM_ID,            HandleIfsArmCgg1},
    {IFS_ARM_CGG2_RXSM_ID,            HandleIfsArmCgg2},
    {IFS_FIRE_BW1_RXSM_ID,            HandleIfsFireBw1},
    {IFS_FIRE_BW2_RXSM_ID,            HandleIfsFireBw2},
    {IFS_FIRE_CGG1_RXSM_ID,           HandleIfsFireCgg1},
    {IFS_FIRE_CGG2_RXSM_ID,           HandleIfsFireCgg2},
    {IFS_ACTUATOR_RESET_RXSM_ID,      HandleIfsActuatorReset},

    {UHFCOM_PING_RXSM_ID,             HandleUhfcomPing},
    {UHFCOM_BEACON_ENABLE_RXSM_ID,    HandleUhfcomBeaconEnable},
    {UHFCOM_BEACON_DISABLE_RXSM_ID,   HandleUhfcomBeaconDisable},
    {UHFCOM_BEACON_DATA_RXSM_ID,      HandleUhfcomBeaconData},


    {CS_PING_RXSM_ID,                 HandleCsPing},
    {CS_CAMERA1_ENABLE_RXSM_ID,       HandleCsCamera1Enable},
    {CS_CAMERA1_DISABLE_RXSM_ID,      HandleCsCamera1Disable},
    {CS_CAMERA2_ENABLE_RXSM_ID,       HandleCsCamera2Enable},
    {CS_CAMERA2_DISABLE_RXSM_ID,      HandleCsCamera2Disable},
    {CS_SPI_ENABLE_RXSM_ID,           HandleCsSpiEnable},
    {CS_SPI_DISABLE_RXSM_ID,          HandleCsSpiDisable},

    {SYSTEM_SET_TEST_MODE_RXSM_ID,    HandleSetTestMode},
    {SYSTEM_SET_FLIGHT_MODE_RXSM_ID,  HandleSetFlightMode},
    {SIMULATE_LO_RXSM_ID,             HandleSimulateLO},
    {SIMULATE_SODS_RXSM_ID,           HandleSimulateSODS},
    {SIMULATE_SOE_RXSM_ID,            HandleSimulateSOE},
    {SIMULATE_EJECTION_RXSM_ID,       HandleSimulateEjection},
};

/* Private user code ---------------------------------------------------------*/

static void HandleEpsPing(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(EPS_PING_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleEpsBatteriesEnable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(EPS_BATTERIES_ENABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleEpsBatteriesDisable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(EPS_BATTERIES_DISABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleEpsChargingEnable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(EPS_CHARGING_ENABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleEpsChargingDisable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(EPS_CHARGING_DISABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleEpsRailEnable(const RXSM_Telecommand_t *tc){
    uint8_t payload[tc->payload_len];
    memcpy(payload, tc->payload, tc->payload_len);

    // Mirror command over CAN
    send_can_command(EPS_RAIL_ENABLE_CAN_ID, payload, 1);
}

static void HandleEpsRailDisable(const RXSM_Telecommand_t *tc){
    uint8_t payload[tc->payload_len];
    memcpy(payload, tc->payload, tc->payload_len);
    
    // Mirror command over CAN
    send_can_command(EPS_RAIL_DISABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleEpsPowerCycle(const RXSM_Telecommand_t *tc){
    uint8_t payload[tc->payload_len];
    memcpy(payload, tc->payload, tc->payload_len);
    
    // Mirror command over CAN
    send_can_command(EPS_POWER_CYCLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsPing(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_PING_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsArmBw1(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_ARM_BW1_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsArmBw2(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_ARM_BW2_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsArmCgg1(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_ARM_CGG1_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsArmCgg2(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_ARM_CGG2_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsFireBw1(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_FIRE_BW1_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsFireBw2(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_FIRE_BW2_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsFireCgg1(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_FIRE_CGG1_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsFireCgg2(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_FIRE_CGG2_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleIfsActuatorReset(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(IFS_ACTUATOR_RESET_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleUhfcomPing(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(UHFCOM_PING_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleUhfcomBeaconEnable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(UHFCOM_BEACON_ENABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleUhfcomBeaconDisable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(UHFCOM_BEACON_DISABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleUhfcomBeaconData(const RXSM_Telecommand_t *tc){
    uint8_t payload[tc->payload_len];
    memcpy(payload, tc->payload, tc->payload_len);
    
    // Mirror command over CAN
    send_can_command(UHFCOM_BEACON_DATA_CAN_ID, payload, 1);
}

static void HandleCsPing(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(CS_PING_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleCsCamera1Enable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(CS_CAMERA1_ENABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleCsCamera1Disable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(CS_CAMERA1_DISABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleCsCamera2Enable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(CS_CAMERA2_ENABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleCsCamera2Disable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(CS_CAMERA2_DISABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleCsSpiEnable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(CS_SPI_ENABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleCsSpiDisable(const RXSM_Telecommand_t *tc){
    (void)tc;
    
    // Mirror command over CAN
    send_can_command(CS_SPI_DISABLE_CAN_ID, (uint8_t[]){0x00}, 1);
}

static void HandleSetTestMode(const RXSM_Telecommand_t *tc){
    (void)tc;

    mission_mode = MISSION_MODE_TEST;

    uint32_t flag = osThreadFlagsSet(SysOrchestratorHandle, SYSTEM_MODE_SELECTED_FLAG);

    if ((int32_t)flag < 0){
        RXSM_SendMessage(SYSTEM_SET_TEST_MODE_RXSM_ID, (uint8_t[]){0x00}, 1); // NOK reply
    } else {
        RXSM_SendMessage(SYSTEM_SET_TEST_MODE_RXSM_ID, (uint8_t[]){0xFF}, 1); // OK reply
    }
}

static void HandleSetFlightMode(const RXSM_Telecommand_t *tc){
    (void)tc;

    mission_mode = MISSION_MODE_FLIGHT;

    uint32_t flag = osThreadFlagsSet(SysOrchestratorHandle, SYSTEM_MODE_SELECTED_FLAG);

    if ((int32_t)flag < 0){
        RXSM_SendMessage(SYSTEM_SET_FLIGHT_MODE_RXSM_ID, (uint8_t[]){0x00}, 1); // NOK reply
    } else {
        RXSM_SendMessage(SYSTEM_SET_FLIGHT_MODE_RXSM_ID, (uint8_t[]){0xFF}, 1); // OK reply
    }
}

static void HandleSimulateLO(const RXSM_Telecommand_t *tc)
{
    (void)tc;

    uint32_t flag = osThreadFlagsSet(SysOrchestratorHandle, LO_VALID_EDGE_FLAG);

    if ((int32_t)flag < 0){
        RXSM_SendMessage(SIMULATE_LO_RXSM_ID, (uint8_t[]){0x00}, 1); // NOK reply
    } else {
        RXSM_SendMessage(SIMULATE_LO_RXSM_ID, (uint8_t[]){0xFF}, 1); // OK reply
    }
}

static void HandleSimulateSODS(const RXSM_Telecommand_t *tc)
{
    (void)tc;

     uint32_t flag = osThreadFlagsSet(SysOrchestratorHandle, SODS_VALID_EDGE_FLAG);

    if ((int32_t)flag < 0){
        RXSM_SendMessage(SIMULATE_SODS_RXSM_ID, (uint8_t[]){0x00}, 1); // NOK reply
    } else {
        RXSM_SendMessage(SIMULATE_SODS_RXSM_ID, (uint8_t[]){0xFF}, 1); // OK reply
    }
}

static void HandleSimulateSOE(const RXSM_Telecommand_t *tc)
{
    (void)tc;

     uint32_t flag = osThreadFlagsSet(SysOrchestratorHandle, SOE_VALID_EDGE_FLAG);

    if ((int32_t)flag < 0){
        RXSM_SendMessage(SIMULATE_SOE_RXSM_ID, (uint8_t[]){0x00}, 1); // NOK reply
    } else {
        RXSM_SendMessage(SIMULATE_SOE_RXSM_ID, (uint8_t[]){0xFF}, 1); // OK reply
    }
}

static void HandleSimulateEjection(const RXSM_Telecommand_t *tc)
{
    (void)tc;

     uint32_t flag = osThreadFlagsSet(SysOrchestratorHandle, EJECTION_VALID_EDGE_FLAG);

    if ((int32_t)flag < 0){
        RXSM_SendMessage(SIMULATE_EJECTION_RXSM_ID, (uint8_t[]){0x00}, 1); // NOK reply
    } else {
        RXSM_SendMessage(SIMULATE_EJECTION_RXSM_ID, (uint8_t[]){0xFF}, 1); // OK reply
    }
}

static uint16_t RXSM_ComputeCRC(const RXSM_Telecommand_t *tc)
{
    uint8_t buf[2 + 2 + 1 + RXSM_TC_MAX_PAYLOAD];
    uint16_t i = 0;

    buf[i++] = RXSM_TC_SYNC1;
    buf[i++] = RXSM_TC_SYNC2;

    buf[i++] = (tc->msg_id >> 8) & 0xFF;
    buf[i++] = (tc->msg_id) & 0xFF;

    buf[i++] = tc->payload_len;

    memcpy(&buf[i], tc->payload, tc->payload_len);
    i += tc->payload_len;

    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buf, i);

    return (uint16_t)crc;
}

/* Public user code ---------------------------------------------------------*/
void RXSM_Init(void)
{
    rxsm_rx_head = 0;
    rxsm_rx_tail = 0;

    parser.state = RX_WAIT_SYNC1;
    parser.cmd_id = 0;
    parser.len = 0;
    parser.index = 0;
    parser.crc = 0;
}

void RXSM_PushByte(uint8_t byte){
    uint16_t next = (rxsm_rx_head + 1) % RXSM_RX_BUFFER_SIZE;

    if (next != rxsm_rx_tail)
    {
        rxsm_rx_buffer[rxsm_rx_head] = byte;
        rxsm_rx_head = next;
    }
}

bool RXSM_GetMessage(RXSM_Telecommand_t *tc){
    while (rxsm_rx_tail != rxsm_rx_head)
    {
        uint8_t byte = rxsm_rx_buffer[rxsm_rx_tail];
        rxsm_rx_tail = (rxsm_rx_tail + 1) % RXSM_RX_BUFFER_SIZE;

        switch (parser.state)
        {
            case RX_WAIT_SYNC1:
                if (byte == RXSM_TC_SYNC1)
                {
                    parser.state = RX_WAIT_SYNC2;
                }
                break;

            case RX_WAIT_SYNC2:
                if (byte == RXSM_TC_SYNC2)
                {
                    parser.state = RX_WAIT_CMD_H;
                }
                else if (byte == RXSM_TC_SYNC1) // Avoid missed syncronization in case of "SYNC1 SYNC1 SYNC2"
                {
                    parser.state = RX_WAIT_SYNC2;
                }
                else
                {
                    parser.state = RX_WAIT_SYNC1;
                }
                break;

            case RX_WAIT_CMD_H:
                parser.cmd_id = ((uint16_t)byte << 8);
                parser.state = RX_WAIT_CMD_L;
                break;

            case RX_WAIT_CMD_L:
                parser.cmd_id |= byte;
                parser.state = RX_WAIT_LEN;
                break;

            case RX_WAIT_LEN:
                parser.len = byte;
                parser.index = 0;

                if (parser.len > sizeof(parser.payload)) {
                    parser.state = RX_WAIT_SYNC1;
                } else if (parser.len == 0)  {
                    parser.state = RX_WAIT_CRC_H;
                } else {
                    parser.state = RX_WAIT_PAYLOAD;
                }
                break;

            case RX_WAIT_PAYLOAD:
                parser.payload[parser.index++] = byte;

                if (parser.index >= parser.len) {
                    parser.state = RX_WAIT_CRC_H;
                }
                break;

            case RX_WAIT_CRC_H:
                parser.crc = ((uint16_t)byte << 8);
                parser.state = RX_WAIT_CRC_L;
                break;

            case RX_WAIT_CRC_L:
                parser.crc |= byte;
                
                tc->msg_id = parser.cmd_id;
                tc->payload_len = parser.len;
                memcpy(tc->payload, parser.payload, parser.len);
                
                parser.state = RX_WAIT_SYNC1;

                uint16_t crc_calc = RXSM_ComputeCRC(tc);

                if (crc_calc != parser.crc)
                {
                    break;
                }

                return true;
        }
    }

    return false;

}

void RXSMInterface_ProcessMessage(const RXSM_Telecommand_t *tc){
    
    for (size_t i = 0; i < ARRAY_SIZE(dispatch_table); i++)
    {
        if (dispatch_table[i].id == tc->msg_id)
        {
            dispatch_table[i].handler(tc);
            return;
        }
    }
}

int RXSM_SendMessage(const uint16_t id, const uint8_t *payload, const uint8_t len)
{
    if (len > RXSM_TC_MAX_PAYLOAD || (payload == NULL && len > 0))
    {
        return -1;
    }

    uint16_t idx = 0;

    // Sync
    rxsm_tx_frame[idx++] = RXSM_TC_SYNC1;
    rxsm_tx_frame[idx++] = RXSM_TC_SYNC2;

    // ID
    rxsm_tx_frame[idx++] = (uint8_t)(id >> 8);
    rxsm_tx_frame[idx++] = (uint8_t)(id & 0xFF);

    // Length
    rxsm_tx_frame[idx++] = len;

    // Payload
    if (len > 0)
    {
        memcpy(&rxsm_tx_frame[idx], payload, len);
        idx += len;
    }

    // CRC
    uint32_t crc_input_len = RXSM_TC_OVERHEAD_BYTES - 2 + len;
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)rxsm_tx_frame, crc_input_len);

    rxsm_tx_frame[idx++] = (uint8_t)(crc >> 8);
    rxsm_tx_frame[idx++] = (uint8_t)(crc & 0xFF);

    // Send out via UART
    if (HAL_UART_Transmit_IT(&hlpuart1, rxsm_tx_frame, idx) != HAL_OK){
        return -1;
    }

    return 0;
}
