#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include "main.h"
#include "stm32l4xx_hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TRANSMIT CAN IDs */
/* Commands that are transmitted over CAN */

#define EPS_PING_CAN_ID 0x100
#define EPS_BATTERIES_ENABLE_CAN_ID 0x101
#define EPS_BATTERIES_DISABLE_CAN_ID 0x102
#define EPS_CHARGING_ENABLE_CAN_ID 0x103
#define EPS_CHARGING_DISABLE_CAN_ID 0x104
#define EPS_RAIL_ENABLE_CAN_ID 0x105
#define EPS_RAIL_DISABLE_CAN_ID 0x106
#define EPS_POWER_CYCLE_CAN_ID 0x107

#define IFS_PING_CAN_ID 0x230
#define IFS_ARM_BW1_CAN_ID 0x231
#define IFS_ARM_BW2_CAN_ID 0x232
#define IFS_ARM_CGG1_CAN_ID 0x233
#define IFS_ARM_CGG2_CAN_ID 0x234
#define IFS_FIRE_BW1_CAN_ID 0x235
#define IFS_FIRE_BW2_CAN_ID 0x236
#define IFS_FIRE_CGG1_CAN_ID 0x237
#define IFS_FIRE_CGG2_CAN_ID 0x238
#define IFS_ACTUATOR_RESET_CAN_ID 0x239

#define UHFCOM_PING_CAN_ID 0x360
#define UHFCOM_BEACON_ENABLE_CAN_ID 0x361
#define UHFCOM_BEACON_DISABLE_CAN_ID 0x362
#define UHFCOM_BEACON_DATA_CAN_ID 0x363

#define CS_PING_CAN_ID 0x490
#define CS_CAMERA1_ENABLE_CAN_ID 0x491
#define CS_CAMERA1_DISABLE_CAN_ID 0x492
#define CS_CAMERA2_ENABLE_CAN_ID 0x493
#define CS_CAMERA2_DISABLE_CAN_ID 0x494
#define CS_SPI_ENABLE_CAN_ID 0x495
#define CS_SPI_DISABLE_CAN_ID 0x496

/* Reply CAN IDs for transmitted commands */

#define EPS_PING_CAN_REPLY_ID 0x000
#define EPS_BATTERIES_ENABLE_CAN_REPLY_ID 0x001
#define EPS_BATTERIES_DISABLE_CAN_REPLY_ID 0x002
#define EPS_CHARGING_ENABLE_CAN_REPLY_ID 0x003
#define EPS_CHARGING_DISABLE_CAN_REPLY_ID 0x004
#define EPS_RAIL_ENABLE_CAN_REPLY_ID 0x005
#define EPS_RAIL_DISABLE_CAN_REPLY_ID 0x006
#define EPS_POWER_CYCLE_CAN_REPLY_ID 0x007

#define IFS_PING_CAN_REPLY_ID 0x030
#define IFS_ARM_BW1_CAN_REPLY_ID 0x031
#define IFS_ARM_BW2_CAN_REPLY_ID 0x032
#define IFS_ARM_CGG1_CAN_REPLY_ID 0x033
#define IFS_ARM_CGG2_CAN_REPLY_ID 0x034
#define IFS_FIRE_BW1_CAN_REPLY_ID 0x035
#define IFS_FIRE_BW2_CAN_REPLY_ID 0x036
#define IFS_FIRE_CGG1_CAN_REPLY_ID 0x037
#define IFS_FIRE_CGG2_CAN_REPLY_ID 0x038
#define IFS_ACTUATOR_RESET_CAN_REPLY_ID 0x039

#define UHFCOM_PING_CAN_REPLY_ID 0x060
#define UHFCOM_BEACON_ENABLE_CAN_REPLY_ID 0x061
#define UHFCOM_BEACON_DISABLE_CAN_REPLY_ID 0x062
#define UHFCOM_BEACON_DATA_CAN_REPLY_ID 0x063

#define CS_PING_CAN_REPLY_ID 0x090
#define CS_CAMERA1_ENABLE_CAN_REPLY_ID 0x091
#define CS_CAMERA1_DISABLE_CAN_REPLY_ID 0x092
#define CS_CAMERA2_ENABLE_CAN_REPLY_ID 0x093
#define CS_CAMERA2_DISABLE_CAN_REPLY_ID 0x094
#define CS_SPI_ENABLE_CAN_REPLY_ID 0x095
#define CS_SPI_DISABLE_CAN_REPLY_ID 0x096

/* ACCEPTS */
/* Commands that are accepted by the system */
#define EPS_RADIO_SILENCE_ACK_CAN_ID 0x02F

#define IFS_WAKE_UP_CAN_ID 0x05F
#define IFS_ARM_DECAY_CAN_ID 0x05E
#define IFS_FIRE_DECAY_CAN_ID 0x05D
#define IFS_REP_ACTUATOR_SPENT_CAN_ID 0x05C

#define UHFCOM_WAKE_UP_CAN_ID 0x08F

#define CS_WAKE_UP_CAN_ID 0x0BF
#define CS_POWER_CYCLE_CAN_ID 0x0BE

/* Reply CAN IDs for accepted commands */
#define EPS_RADIO_SILENCE_ACK_CAN_REPLY_ID 0x12F

#define IFS_WAKE_UP_CAN_REPLY_ID 0x25F
#define IFS_ARM_DECAY_CAN_REPLY_ID 0x25E
#define IFS_FIRE_DECAY_CAN_REPLY_ID 0x25D
#define IFS_REP_ACTUATOR_SPENT_CAN_REPLY_ID 0x25C

#define UHFCOM_WAKE_UP_CAN_REPLY_ID 0x38F

#define CS_WAKE_UP_CAN_REPLY_ID 0x4BF

/* Data CAN IDs */
#define GNSS_POSITION_CAN_ID 0x503


/** Maximum number of simultaneously tracked outgoing commands. */
#define PENDING_CMD_MAX 16
 
/** Time without a reply before the first retransmission (ms). */
#define PENDING_CMD_TIMEOUT_MS 1000

/* Data structure for received CAN messages, includes both the header and data payload */
typedef struct {
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
} can_rx_msg_t;

/**
 * @brief Send a CAN command with the specified ID, data, and data length.
 * @param id: CAN ID
 * @param data: Pointer to the data to send
 * @param dlc: Data length (DLC)
 * @retval None
 */
void send_can_command(uint16_t id, const uint8_t *data, uint8_t dlc);

/**
 * @brief  Send a CAN command and register it for timeout-based retry.
 * @param  cmd_id    CAN ID to transmit.
 * @param  reply_id  CAN ID expected in reply.
 * @param  data      Payload bytes.
 * @param  dlc       Payload length (0–8).
 */
void send_can_command_tracked(uint16_t cmd_id, uint16_t reply_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief  Initialise the pending-command subsystem.
 *         Call once before the RTOS scheduler starts, or from an init task.
 */
void can_pending_init(void);

/**
 * @brief  Cancel the pending entry whose expected reply matches reply_id.
 *         Call at the top of CommandInterface_ProcessMessage().
 */
void can_pending_clear(uint16_t reply_id);
 
/**
 * @brief  Scan the pending table and retransmit any timed-out commands.
 *         Call periodically from the CommandInterface task loop.
 */
void can_pending_retry(void);


#ifdef __cplusplus
}
#endif

#endif /* CAN_H */