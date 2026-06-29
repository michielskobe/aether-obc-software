#ifndef RXSM_INTERFACE_H
#define RXSM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "can.h"

/* =========================================================================
 * REXUS Packet Format (Figure 7-1)
 *
 *  | SYNC1 | SYNC2 | MSGID_H | MSGID_L | MSGLEN | Data 0..N  | CRC_H | CRC_L |
 *  |  1 B  |  1 B  |   1 B   |   1 B   |   1 B  |   0..N B   |  1 B  |  1 B  |
 *
 *  MSGID is 16-bit (big-endian) to directly carry CAN IDs (up to 0x7FF).
 *  Total packet must not exceed 64 bytes (RXSM buffer limit).
 *  UART: 38400 baud, 8N1, RS-422 physical layer.
 *
 *  RX (RXSM → FFU): incoming telecommands.
 *  TX (FFU → RXSM): outbound status/ACK/error.
 *  The two directions are on separate UART lines; IDs are reused across
 *  directions by convention — the same numeric ID means "command" on RX
 *  and "result of that command" on TX.
 * ========================================================================= */
 
/* Sync word */
#define RXSM_TC_SYNC1                0xAA
#define RXSM_TC_SYNC2                0x55
 
/* Overhead: SYNC1 + SYNC2 + MSGID(2) + MCNT + + CRC(2) */
#define RXSM_TC_OVERHEAD_BYTES       7U
#define RXSM_TC_MAX_PACKET           64U
#define RXSM_TC_MAX_PAYLOAD          (RXSM_TC_MAX_PACKET - RXSM_TC_OVERHEAD_BYTES)

/* Packet field offsets */
#define RXSM_TC_OFFSET_SYNC1         0U
#define RXSM_TC_OFFSET_SYNC2         1U
#define RXSM_TC_OFFSET_MSGID_H       2U
#define RXSM_TC_OFFSET_MSGID_L       3U
#define RXSM_TC_OFFSET_MSGLEN        4U
#define RXSM_TC_OFFSET_DATA          5U

/* EPS IDs — mirrored CAN IDs */
#define EPS_PING_RXSM_ID                 EPS_PING_CAN_ID
#define EPS_BATTERIES_ENABLE_RXSM_ID     EPS_BATTERIES_ENABLE_CAN_ID
#define EPS_BATTERIES_DISABLE_RXSM_ID    EPS_BATTERIES_DISABLE_CAN_ID
#define EPS_CHARGING_ENABLE_RXSM_ID      EPS_CHARGING_ENABLE_CAN_ID
#define EPS_CHARGING_DISABLE_RXSM_ID     EPS_CHARGING_DISABLE_CAN_ID
#define EPS_RAIL_ENABLE_RXSM_ID          EPS_RAIL_ENABLE_CAN_ID
#define EPS_RAIL_DISABLE_RXSM_ID         EPS_RAIL_DISABLE_CAN_ID
#define EPS_POWER_CYCLE_RXSM_ID          EPS_POWER_CYCLE_CAN_ID
 
/* IFS IDs — mirrored CAN IDs */
#define IFS_PING_RXSM_ID                 IFS_PING_CAN_ID
#define IFS_ARM_BW1_RXSM_ID              IFS_ARM_BW1_CAN_ID
#define IFS_ARM_BW2_RXSM_ID              IFS_ARM_BW2_CAN_ID
#define IFS_ARM_CGG1_RXSM_ID             IFS_ARM_CGG1_CAN_ID
#define IFS_ARM_CGG2_RXSM_ID             IFS_ARM_CGG2_CAN_ID
#define IFS_FIRE_BW1_RXSM_ID             IFS_FIRE_BW1_CAN_ID
#define IFS_FIRE_BW2_RXSM_ID             IFS_FIRE_BW2_CAN_ID
#define IFS_FIRE_CGG1_RXSM_ID            IFS_FIRE_CGG1_CAN_ID
#define IFS_FIRE_CGG2_RXSM_ID            IFS_FIRE_CGG2_CAN_ID
#define IFS_ACTUATOR_RESET_RXSM_ID       IFS_ACTUATOR_RESET_CAN_ID
 
/* UHFCOM IDs — mirrored CAN IDs */
#define UHFCOM_PING_RXSM_ID              UHFCOM_PING_CAN_ID
#define UHFCOM_BEACON_ENABLE_RXSM_ID     UHFCOM_BEACON_ENABLE_CAN_ID
#define UHFCOM_BEACON_DISABLE_RXSM_ID    UHFCOM_BEACON_DISABLE_CAN_ID
#define UHFCOM_BEACON_DATA_RXSM_ID       UHFCOM_BEACON_DATA_CAN_ID
 
/* CS IDs — mirrored CAN IDs */
#define CS_PING_RXSM_ID                  CS_PING_CAN_ID
#define CS_CAMERA1_ENABLE_RXSM_ID        CS_CAMERA1_ENABLE_CAN_ID
#define CS_CAMERA1_DISABLE_RXSM_ID       CS_CAMERA1_DISABLE_CAN_ID
#define CS_CAMERA2_ENABLE_RXSM_ID        CS_CAMERA2_ENABLE_CAN_ID
#define CS_CAMERA2_DISABLE_RXSM_ID       CS_CAMERA2_DISABLE_CAN_ID
#define CS_SPI_ENABLE_RXSM_ID            CS_SPI_ENABLE_CAN_ID
#define CS_SPI_DISABLE_RXSM_ID           CS_SPI_DISABLE_CAN_ID

#define RXSM_RX_BUFFER_SIZE 128

typedef struct {
    uint16_t msg_id;                        /* Command ID               */
    uint8_t  payload_len;                   /* Number of valid payload bytes */
    uint8_t  payload[RXSM_TC_MAX_PAYLOAD];  /* Command-specific data        */
} RXSM_Telecommand_t;

void RXSM_Init(void);

/**
 * @brief Process a received UART byte.
 *
 * Called from HAL_UART_RxCpltCallback().
 *
 * @param byte Received byte.
 */
void RXSM_PushByte(uint8_t byte);

/**
 * @brief Retrieve a complete message if available.
 *
 * @param msg Output message.
 *
 * @retval true  A complete message was returned.
 * @retval false No message available.
 */
bool RXSM_GetMessage(RXSM_Telecommand_t *tc);

/**
 * @brief  Dispatch an incoming RXSM message to the appropriate handler.
 *         Call from the RMU Manager task whenever a message is dequeued.
 */

void RXSMInterface_ProcessMessage(const RXSM_Telecommand_t *tc);

#ifdef __cplusplus
}
#endif

#endif /* RXSM_INTERFACE_H */
