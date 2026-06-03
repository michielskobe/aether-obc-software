/**
  ******************************************************************************
  * @file           : mosaic_x5.c
  * @brief          : Implementation for mosaic_x5.h
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
#include "mosaic_x5.h"
#include "nmea.h"
#include "data_packet.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "can.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define GNSS_RX_BUF_SIZE 512
#define GNSS_LINE_BUF_SIZE 128

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart5; // UART handle declared in main.c, used for receiving data from the mosaic-X5 GNSS module
extern osThreadId_t GNSSManagerHandle; // Declared in main.c, used to set flags from mosaic-X5 data parser
extern osMessageQueueId_t SD_CardQueueHandle; // Declared in main.c, used to put data packets into the SDCardManager thread for writing to SD card
extern osMessageQueueId_t IridiumQueueHandle; // Declared in main.c, used to put data packets into the IridiumManager thread for transmission over Iridium
extern osMessageQueueId_t MissionPhaseDataQueueHandle; // Declared in main.c, used to put data packets into the SystemOrchestrator thread for mission phase management based on altitude
extern metadata_t mission_metadata; // Declared in main.c, used to store mission metadata such as the latest GNSS coordinates
uint8_t gnss_rx_buf[GNSS_RX_BUF_SIZE];
volatile uint16_t gnss_rx_size = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* BARE-MINIMUM LINE CONSUMER: Reads lines out of the circular buffer */
static void gnss_data_parser(void){
    static char linebuf[GNSS_LINE_BUF_SIZE];
    static uint16_t line_idx = 0;
    static uint8_t counter = 0;

    for (uint16_t i = 0; i < gnss_rx_size; i++) {
        // Read a character from the DMA buffer
        char c = (char)gnss_rx_buf[i];

        // Frame synchronization
        if (c == '$') {
            linebuf[0] = '$';
            line_idx = 1;
            continue;
        }

        // If we haven't seen a start delimiter yet, ignore garbage bytes
        if (line_idx == 0) continue;

        if(c == '\r' || c == '\n') {
            if(line_idx > 1) {
                linebuf[line_idx] = '\0';

                nmea_data_t nmea = {0}; // Zero-initialize structure
                // Non-NMEA sentences (like command replies) are automatically ignored here             
                if (nmea_parse_sentence(linebuf, &nmea) == 0) {
                    data_packet_t p_lat, p_lon, p_alt;
                    uint32_t tick = osKernelGetTickCount();

                    // Pack timestamps
                    p_lat.timestamp[0] = p_lon.timestamp[0] = p_alt.timestamp[0] = (tick >> 16) & 0xFF;
                    p_lat.timestamp[1] = p_lon.timestamp[1] = p_alt.timestamp[1] = (tick >> 8) & 0xFF;
                    p_lat.timestamp[2] = p_lon.timestamp[2] = p_alt.timestamp[2] = tick & 0xFF;

                    // Pack IDs
                    p_lat.id = (uint8_t)0x500; 
                    p_lon.id = (uint8_t)0x501; 
                    p_alt.id = (uint8_t)0x502;

                    /* Latitude packet: [lat_b2, lat_b1, lat_b0, fix_quality] */
                    p_lat.data[0] = (nmea.latitude >> 16) & 0xFF;  
                    p_lat.data[1] = (nmea.latitude >> 8) & 0xFF;
                    p_lat.data[2] = nmea.latitude & 0xFF;          
                    p_lat.data[3] = nmea.fix_quality;

                    /* Longitude packet: [lon_b2, lon_b1, lon_b0, satellites] */
                    p_lon.data[0] = (nmea.longitude >> 16) & 0xFF;
                    p_lon.data[1] = (nmea.longitude >>  8) & 0xFF;
                    p_lon.data[2] =  nmea.longitude & 0xFF;
                    p_lon.data[3] =  nmea.satellites;

                    /* Altitude packet: [alt_MSB, alt_LSB, hdop_x10, 0x00] */
                    p_alt.data[0] = (nmea.altitude >> 8) & 0xFF;
                    p_alt.data[1] =  nmea.altitude & 0xFF;
                    p_alt.data[2] =  nmea.hdop_x10;
                    p_alt.data[3] =  counter++; // Just a counter to have some changing data in the last byte for testing. (TODO: remove)

                    // Put the data packets into the SD_CardQueue for processing by the SDCardManager thread
                    osMessageQueuePut(SD_CardQueueHandle, &p_lat, 0, 0);
                    osMessageQueuePut(SD_CardQueueHandle, &p_lon, 0, 0);
                    osMessageQueuePut(SD_CardQueueHandle, &p_alt, 0, 0);

                    // Put the data packets into the IridiumQueue as well for transmission over Iridium by the IridiumManager thread 
                    osMessageQueuePut(IridiumQueueHandle, &p_lat, 0, 0);
                    osMessageQueuePut(IridiumQueueHandle, &p_lon, 0, 0);
                    osMessageQueuePut(IridiumQueueHandle, &p_alt, 0, 0);

                    // Put altitude packet in the MissionPhaseDataQueue for use in mission phase management by the SystemOrchestrator
                    can_rx_msg_t msg_alt;
                    msg_alt.RxHeader.StdId = 0x502; // CAN ID for altitude
                    msg_alt.RxHeader.DLC = 2;
                    memset(msg_alt.RxData, 0, sizeof(msg_alt.RxData));
                    memcpy(msg_alt.RxData, p_alt.data, 2); // Copy the altitude data
                    osMessageQueuePut(MissionPhaseDataQueueHandle, &msg_alt, 0, 0);

                    // Reformat the data into a 8-byte payload for the UHFCOM
                    uint8_t gnss_payload[8];
                    gnss_payload[0] = (nmea.latitude >> 16) & 0xFF;  
                    gnss_payload[1] = (nmea.latitude >> 8) & 0xFF;
                    gnss_payload[2] =  nmea.latitude & 0xFF;
                    gnss_payload[3] = (nmea.longitude >> 16) & 0xFF;
                    gnss_payload[4] = (nmea.longitude >>  8) & 0xFF;
                    gnss_payload[5] =  nmea.longitude & 0xFF;
                    gnss_payload[6] = (nmea.altitude >> 8) & 0xFF;
                    gnss_payload[7] =  nmea.altitude & 0xFF;

                    // Send the GNSS data to the UHFCOM
                    send_can_command(GNSS_POSITION_CAN_ID, gnss_payload, 8);

                    // Check if GNSS data is valid
                    if (nmea.fix_quality != 0) {
                        // If valid, store the latest GNSS data in the mission metadata
                        memcpy(mission_metadata.gnss, gnss_payload, 8);
                    }
                }
            }
            line_idx = 0; // Reset for next line
        } else {
            if(line_idx < GNSS_LINE_BUF_SIZE-1) {
                linebuf[line_idx++] = c;
            } else {
                line_idx = 0;
            }
        }
    }
}

/* Public user code ----------------------------------------------------------*/

void gnss_data_handler(void){
    // Wait for the flag indicating that new GNSS data is available
    osThreadFlagsWait(GNSS_DATA_AVAILABLE, osFlagsWaitAny, osWaitForever);

    // Handle the received data
    gnss_data_parser();

    // Restart the UART DMA for the next batch of data
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, gnss_rx_buf, GNSS_RX_BUF_SIZE);
}

int mosaic_x5_init(void){
    /* The mosaic-X5 has a boot configuration stored in non-volatile memory that is loaded on power-up.
     * This boot configuration includes setting the mosaic-X5 to output GGA sentences every 2 seconds on COM1 (setNMEAOutput),
     * setting the mosaic-X5's receiver dynamics to "Unlimited" (setReceiverDynamics), 
     *  and setting the mosaic-X5's satellite tracking to "All" (setSatelliteTracking).
    */

    // Start interrupt-driven receive for NMEA data
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, gnss_rx_buf, GNSS_RX_BUF_SIZE) != HAL_OK) {
        return -1; // Initialization failed
    }
    
    return 0; // Initialization successful    
}

void mosaic_uart_rx_cb(uint16_t size){
    // Update the global variable with the size of the received data
    gnss_rx_size = size;

    // Set the flag to indicate that new GNSS data is available for processing
    osThreadFlagsSet(GNSSManagerHandle, GNSS_DATA_AVAILABLE);

}