/**
  ******************************************************************************
  * @file           : data_acquisition.c
  * @brief          : Implementation for data_acquisition.h
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
#include "cmsis_os.h"
#include "data_acquisition.h"
#include "data_packet.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern osMessageQueueId_t SD_CardQueueHandle; // Declared in main.c

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Process a CAN data message and split it into multiple data packets if necessary.
 *        For payloads > 4 bytes, creates multiple packets with sequentially incremented IDs.
 * @param rx_msg: Pointer to received CAN message
 * @param timestamp: 3-byte timestamp array to use for all packets
 * @param base_id: Base identifier (lower 8 bits of CAN ID)
 * @retval None
 */
static void process_can_data_message(const can_rx_msg_t *rx_msg, const uint8_t *timestamp, uint8_t base_id)
{
  uint8_t data_length = rx_msg->RxHeader.DLC;
  if (data_length > 8)
  {
    return; // Reject malformed frames
  }
  uint8_t packet_count = (data_length <= 4) ? 1 : 2;  // Calculate number of 4-byte packets needed
  
  for (uint8_t packet_idx = 0; packet_idx < packet_count; packet_idx++)
  {
    data_packet_t packet;
    
    memcpy(packet.timestamp, timestamp, 3);

    packet.id = base_id + packet_idx;
    
    uint8_t bytes_to_copy = data_length - (packet_idx * 4);
    if (bytes_to_copy > 4) {bytes_to_copy = 4;}
    
    memset(packet.data, 0, 4);
    memcpy(packet.data, &rx_msg->RxData[packet_idx * 4], bytes_to_copy);
    
    osMessageQueuePut(SD_CardQueueHandle, &packet, 0, 0);
  }
}

/* Public user code ---------------------------------------------------------*/

void DataAcquisition_ProcessMessage(const can_rx_msg_t *msg) {
  if (msg == NULL) {return;}

  // Get the current tick count to use as a timestamp for the data packet
    uint32_t tick = osKernelGetTickCount();

    uint8_t timestamp[3] = 
    {
        (tick >> 16) & 0xFF,
        (tick >> 8)  & 0xFF,
        tick & 0xFF
    };

    // Use the lower 8 bits of the CAN message's standard ID as the base sensor ID
    uint8_t base_id = (uint8_t)(msg->RxHeader.StdId & 0xFF);

    // Process the CAN message and queue all resulting data packets
    process_can_data_message(msg, timestamp, base_id);  
}