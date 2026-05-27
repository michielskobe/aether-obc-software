/**
  ******************************************************************************
  * @file           : sd_spi.c
  * @brief          : Implementation for sd_spi.h
  * @author         : Kobe Michiels
  ******************************************************************************
  * @note
  * The implementation of these functions is mostly  based on the ELM-CHaN website on how to use MMC/SDC
  * (https://elm-chan.org/docs/mmc/mmc_e.html) and Cristinel Ababei's lecture on SPI and SD cards 
  * at Marquette University (https://www.dejazzer.com/coen4720_old/lecture_notes/lec08_sd_cards.pdf).
  *
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "sd_spi.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern CRC_HandleTypeDef hcrc;

static const uint8_t cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};    // CMD0: GO_IDLE_STATE (Software reset)
static const uint8_t cmd1[] = {0x41, 0x00, 0x00, 0x00, 0x00, 0xF9};    // CMD1: SEND_OP_COND (Initiate initialization process)
static const uint8_t acmd41[] = {0x69, 0x40, 0x00, 0x00, 0x00, 0x77};  // ACMD41: APP_SEND_OP_COND (For only SDC. Initiate initialization process)
static const uint8_t cmd8[] = {0x48, 0x00, 0x00, 0x01, 0xAA, 0x87};    // CMD8: SEND_IF_COND (For only SDC V2. Check voltage range)
static const uint8_t cmd55[] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x65};   // CMD55: APP_CMD (Leading command of ACMD<n> command)
static const uint8_t cmd58[] = {0x7A, 0x00, 0x00, 0x00, 0x00, 0xFD};   // CMD58: READ_OCR (Read OCR)
static const uint8_t cmd59[] = {0x7B, 0x00, 0x00, 0x00, 0x00, 0x91};   // CMD59: CRC_ON_OFF (Toggle CRC option)

static const uint8_t high_byte = 0xFF;
/* Private function prototypes -----------------------------------------------*/

/**
* @brief Select the SD card on the SPI bus.
*
* Pulls the Chip Select (CS) GPIO pin low to enable communication
* with the SD card. This must be called before sending any command
* or data over SPI to the SD card.
*
*/
 void sd_select(void);

/**
* @brief Deselect the SD card and release the SPI bus.
*
* This function pulls the Chip Select (CS) line high to deselect
* the SD card, ending the current SPI transaction.
*
* After deselecting, it transmits one dummy byte (0xFF) over SPI
* to generate 8 clock cycles. This ensures the MISO line is properly 
* released (goes high).
*
*/
void sd_deselect(void);

/**
* @brief Send a command to the SD card over SPI and receive its response.
*
* Transmits a command packet to the SD card and reads back the response.
* The function handles the SPI transaction, including waiting for the
* card's response within a timeout period.
*
* @param[in]  cmd       Pointer to the command buffer.
*                       Typically a 6-byte SD command frame:
*                       [command | argument (4 bytes) | CRC].
*
* @param[out] resp      Pointer to a buffer where the response will be stored.
*
* @param[in]  resp_len  Expected length of the response in bytes.
*                       For example:
*                       - 1 byte for R1 responses
*                       - 5 bytes for R3/R7 responses
*
* @return uint8_t
*         - 0 on success (response received correctly)
*         - Non-zero error code on failure
*
*/
uint8_t sd_send_cmd(const uint8_t *cmd, uint8_t *resp, uint8_t resp_len);

/**
* @brief Send an application-specific command (ACMD) to the SD card.
*
* Application-specific commands (ACMD<n>) are issued as a two-step sequence:
*   1. Send CMD55 (APP_CMD) to indicate that the next command is application-specific
*   2. Send the actual command (ACMD<n>)
*
* This function handles both steps internally and returns the response
* from the second command (ACMD<n>).
*
* @note Internally, this function typically performs:
*       - sd_send_cmd(CMD55, ...)
*       - sd_send_cmd(cmd, ...)
*
* @warning If CMD55 fails, the ACMD will not be sent.
*
* @see sd_send_cmd()
*
*/
uint8_t sd_send_acmd(const uint8_t *cmd, uint8_t *resp, uint8_t resp_len);

/**
* @brief Transmit a data packet to the SD card (used for block write operations).
*
* This function sends a complete SD card data packet over SPI, typically used
* after issuing a write command such as CMD24 (WRITE_BLOCK). The packet format is:
*
*   [DATA TOKEN][512-BYTE DATA][2-BYTE CRC]
*
* @param[in] data_token  Pointer to the data token byte.
*
* @param[in] data        Pointer to a 512-byte data buffer to be written.
*
* @param[in] crc         Pointer to a 2-byte CRC.
*                        - Can be dummy (0xFF, 0xFF) if CRC is disabled in SPI mode
*
* @return int
*         - 0 on success (data accepted and written)
*         - Negative value on failure
*
* @see sd_write_block() and sd_write_multiple_block()
*/
int transmit_data_packet(uint8_t *data_token, uint8_t *data, uint8_t *crc);

/**
* @brief Read a data packet from the SD card (used for read operations).
*
* This function gets a complete SD card data packet over SPI, typically used
* after issuing a read command such as CMD17 (READ_SINGLE_BLOCK). The packet format is:
*
*   [DATA TOKEN][512-BYTE DATA][2-BYTE CRC]
*
* @param[out] buffer      Pointer to a 512-byte buffer where the read data
*                         will be stored.
*
* @see sd_read_block()
*/
void read_data_packet(uint8_t *buffer);

/**
* @brief Calculate the CRC7 for a given data buffer.
*
* This function computes the CRC7 checksum for the provided data buffer,
* which is used in SD card communication for error detection.
*
* @param[in] data Pointer to the data buffer.
* @param[in] len  Length of the data buffer in bytes.
*
* @return uint8_t The calculated CRC7 value.
*/
uint8_t crc7_sd(const uint8_t *data, int len);

/* Private user code ---------------------------------------------------------*/

void sd_select(void){
  HAL_GPIO_WritePin(GPIOA, SD_CS_Pin, GPIO_PIN_RESET);
}

void sd_deselect(void){
  HAL_GPIO_WritePin(GPIOA, SD_CS_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit(&hspi1, &high_byte, 1, HAL_MAX_DELAY); // 8 clocks
}

uint8_t sd_send_cmd(const uint8_t *cmd, uint8_t *resp, uint8_t resp_len){
  // CS signal must be driven high to low prior to sending a command frame and kept low during the transaction
  sd_select();

  // Send command
  HAL_SPI_Transmit(&hspi1, cmd, 6, HAL_MAX_DELAY);

  // Wait for R1 response
  uint8_t r1 = 0xFF;
  do {
      HAL_SPI_TransmitReceive(&hspi1, &high_byte, &r1, 1, HAL_MAX_DELAY);
  } while (r1 == 0xFF);

  resp[0] = r1;

  // If response length > 1, read extra bytes (R3/R7)
  for (int i = 1; i < resp_len; i++){
    HAL_SPI_TransmitReceive(&hspi1, &high_byte, &resp[i], 1, HAL_MAX_DELAY);
  }

  // Set CS high and send 8 clock cycles
  sd_deselect();
  
  return r1;
}

uint8_t sd_send_acmd(const uint8_t *cmd, uint8_t *resp, uint8_t resp_len){
  uint8_t cmd55_resp[1];
  if (sd_send_cmd(cmd55, cmd55_resp, 1) != 0x01) {
    return -1; // CMD55 failed
  }

  // Send ACMD command
  return sd_send_cmd(cmd, resp, 1);
}

int transmit_data_packet(uint8_t *data_token, uint8_t *data, uint8_t *crc){
  // Send Data Token
  HAL_SPI_Transmit(&hspi1, data_token, 1, HAL_MAX_DELAY);

  // Send Data Block
  HAL_SPI_Transmit_DMA(&hspi1, data, SD_BLOCK_SIZE);

  // Wait for transmission to complete
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){
    osThreadYield();
  }

  // Send CRC
  HAL_SPI_Transmit(&hspi1, crc, 2, HAL_MAX_DELAY);

  // Receive Data Response
  uint8_t response;
  HAL_SPI_TransmitReceive(&hspi1, &high_byte, &response, 1, HAL_MAX_DELAY);

  // Check if data accepted (Data Response LSBs should be 0x05)
  if ((response & 0x1F) != 0x05) {
        return -1; // Data rejected
  }

  // Wait busy
  uint8_t busy;
  do {
      HAL_SPI_TransmitReceive(&hspi1, &high_byte, &busy, 1, HAL_MAX_DELAY);
  } while (busy == 0x00);

  return 0; // Data Packet transmission successful
}

void read_data_packet(uint8_t *buffer){
  // Detect valid data token
  uint8_t data_token;
  do {
      HAL_SPI_TransmitReceive(&hspi1, &high_byte, &data_token, 1, HAL_MAX_DELAY);
  } while (data_token != 0xFE);

  // Receive the following data field
  HAL_SPI_Receive_DMA(&hspi1, buffer, SD_BLOCK_SIZE);

  // Wait for transmission to complete
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){
    osThreadYield();
  }

  // Receives CRC
  uint8_t crc[2];
  HAL_SPI_Receive(&hspi1, crc, 2, HAL_MAX_DELAY);
}

uint8_t crc7_sd(const uint8_t *data, int len){
  uint8_t crc = 0;

  for (int i = 0; i < len; i++) {
      uint8_t d = data[i];

      for (int b = 0; b < 8; b++) {
          crc <<= 1;

          if ((d & 0x80) ^ (crc & 0x80))
              crc ^= 0x09; // polynomial x^7 + x^3 + 1 (0x89 without MSB)

          d <<= 1;
      }
  }

  return crc & 0x7F;
}

/* Public user code ----------------------------------------------------------*/

int sd_init(void){
  // Power ON: set DI (MOSI) and CS high and apply 74 or more clock pulses
  sd_deselect();
  for(int i=0; i<10; i++) {
    HAL_SPI_Transmit(&hspi1, &high_byte, 1, HAL_MAX_DELAY);
  }

  // Send a CMD0 to reset the card
  uint8_t cmd0_resp[1];
  if (sd_send_cmd(cmd0, cmd0_resp, 1) != 0x01) {
    return -1; // CMD0 failed
  }

  // Send CMD59 to enable CRC
  uint8_t cmd59_resp[1];
  if (sd_send_cmd(cmd59, cmd59_resp, 1) != 0x01) {
    return -1; // CMD59 failed
  }

  // Send CMD8 to check voltage range
  uint8_t cmd8_resp[5];
  if (sd_send_cmd(cmd8, cmd8_resp, 5) != 0x01) {
    return -1; // CMD8 failed
  }

  // Send ACMD41 to initiate initialization process
  int retries = 10;
  uint8_t acmd41_resp[1];
  while (sd_send_acmd(acmd41, acmd41_resp, 1) != 0x00 && retries--) {
    // Wait for card to be ready
    osDelay(pdMS_TO_TICKS(1));
  }
  if (retries <= 0) { // ACMD41 failed
    uint8_t cmd1_resp[1];
    if (sd_send_cmd(cmd1, cmd1_resp, 1) != 0x00) {
      return -1; // CMD1 failed
    }
  }

  // Send CMD58 to read OCR
  uint8_t cmd58_resp[5];
  if (sd_send_cmd(cmd58, cmd58_resp, 5) != 0x00) {
    return -1; // CMD58 failed
  }

  // Initialization commands successful, set SPI clock to higher speed for data transfer
  if (HAL_SPI_DeInit(&hspi1) != HAL_OK){Error_Handler();}
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 20 MHz SPI clock
  if (HAL_SPI_Init(&hspi1) != HAL_OK){Error_Handler();}

  return 0; // Initialization successful
}

int sd_write_block(uint32_t block_addr, uint8_t *data){
  // Send CMD24
  uint8_t cmd24[6] = { // CMD24: WRITE_BLOCK (Write a block)
    0x58,
    (block_addr >> 24) & 0xFF,
    (block_addr >> 16) & 0xFF,
    (block_addr >> 8) & 0xFF,
    block_addr & 0xFF,
    0 // Placeholder for CRC7, will be calculated below
  };

  uint8_t cmd_crc = crc7_sd(cmd24, 5);
  cmd24[5] = (cmd_crc << 1) | 1;


  uint8_t resp[1];
  if (sd_send_cmd(cmd24, resp, 1) != 0x00) {
    return -1; // CMD24 failed
  }

  // Calculate CRC
  uint16_t data_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, SD_BLOCK_SIZE);
  uint8_t crc_bytes[2];
  crc_bytes[0] = (data_crc >> 8) & 0xFF;
  crc_bytes[1] = data_crc & 0xFF;

  // Transmit Data Packet
  uint8_t data_token = 0xFE;
  sd_select();
  int  transmission_result = transmit_data_packet(&data_token, data, crc_bytes);
  sd_deselect();
  return (transmission_result != 0) ? -1 : 0;
}

int sd_write_multiple_block(uint32_t start_block_addr, uint8_t *data, uint8_t number_of_blocks){
  // Send CMD25
  uint8_t cmd25[6] = { // CMD25: WRITE_MULTIPLE_BLOCK (Write multiple blocks)
    0x59,
    (start_block_addr >> 24) & 0xFF,
    (start_block_addr >> 16) & 0xFF,
    (start_block_addr >> 8) & 0xFF,
    start_block_addr & 0xFF,
    0 // Placeholder for CRC7, will be calculated below
  };

  uint8_t cmd_crc = crc7_sd(cmd25, 5);
  cmd25[5] = (cmd_crc << 1) | 1;
  
  uint8_t resp[1];
  if (sd_send_cmd(cmd25, resp, 1) != 0x00) {
    return -1; // CMD25 failed
  }

  uint8_t data_token = 0xFC;
  int failed_blocks = 0;
  
  // CS low for entire transaction
  sd_select();
  
  for (int i = 0; i < number_of_blocks; i++){
    // Calculate CRC
    uint16_t data_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data + (i * SD_BLOCK_SIZE), SD_BLOCK_SIZE);
    uint8_t crc_bytes[2];
    crc_bytes[0] = (data_crc >> 8) & 0xFF;
    crc_bytes[1] = data_crc & 0xFF;

    // Transmit Data Packet
    if (transmit_data_packet(&data_token, data + (i * SD_BLOCK_SIZE), crc_bytes) != 0){
      failed_blocks++; // count failed block but continue
    }
  }

  // Send Stop Token
  uint8_t stop_token = 0xFD;
  HAL_SPI_Transmit(&hspi1, &stop_token, 1, HAL_MAX_DELAY);
  
  // Wait busy
  uint8_t busy;
  do {
      HAL_SPI_TransmitReceive(&hspi1, &high_byte, &busy, 1, HAL_MAX_DELAY);
  } while (busy != 0xFF);

  // Release CS
  sd_deselect();

  if (failed_blocks > 0) {
        return -failed_blocks; // negative count of failed blocks
  }

  return 0; // Write successful
}

int sd_read_block(uint32_t block_addr, uint8_t *buffer) {
  // Send CMD17
  uint8_t cmd17[6] = { // CMD17: READ_SINGLE_BLOCK (Read a block)
    0x51,
    (block_addr >> 24) & 0xFF,
    (block_addr >> 16) & 0xFF,
    (block_addr >> 8) & 0xFF,
    block_addr & 0xFF,
    0 // Placeholder for CRC7, will be calculated below
  };

  uint8_t cmd_crc = crc7_sd(cmd17, 5);
  cmd17[5] = (cmd_crc << 1) | 1;

  uint8_t resp[1];
  if (sd_send_cmd(cmd17, resp, 1) != 0x00){
    return -1;
  }

  // Read Data Packet
  sd_select();
  read_data_packet(buffer);
  sd_deselect();

  return 0;
}

int sd_read_multiple_block(uint32_t start_block_addr, uint8_t *buffer, uint8_t number_of_blocks){
  int return_status = 0;

  for (int i = 0; i < number_of_blocks; i++){
    return_status = sd_read_block(start_block_addr + i, buffer + (i * SD_BLOCK_SIZE)); 
  }
  return return_status == 0? 0 : -1;
}

int metadata_write(metadata_t *metadata) {
    // Calculate CRC
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)metadata, sizeof(metadata_t) - sizeof(uint16_t));
    metadata->crc = crc;

    uint8_t sector[SD_BLOCK_SIZE];
    memset(sector, 0x00, SD_BLOCK_SIZE);
    memcpy(sector, metadata, sizeof(metadata_t));

    // Write to all 3 sectors
    int result = 0;
    for (int i = 0; i < 3; i++) {
      result += sd_write_block(i, sector);
    }
    return (result == 0) ? 0 : -1; // Return 0 if all writes successful, else -1
}

int metadata_read(metadata_t *out) {
    metadata_t copies[3];
    bool valid[3] = {false, false, false};

    // Read all 3 sectors
    for (int i = 0; i < 3; i++) {
        uint8_t sector[SD_BLOCK_SIZE];
        memset(sector, 0x00, SD_BLOCK_SIZE);

        
        int result = sd_read_block(i, sector);

        if (result != 0) continue;

        memcpy(&copies[i], sector, sizeof(metadata_t));

        // Validate CRC — compute over all fields except the crc field itself
        uint16_t computed_crc = HAL_CRC_Calculate(&hcrc, 
            (uint32_t *)&copies[i], 
            (sizeof(metadata_t) - sizeof(uint16_t)) / 4); // CRC unit is words
        
        if (computed_crc == copies[i].crc) {
            valid[i] = true;
        }
    }

    // Count valid copies
    int valid_count = valid[0] + valid[1] + valid[2];

    if (valid_count == 0) {
        return -1; // All copies corrupt
    }

    if (valid_count == 1) {
        // Only one valid copy, use it
        for (int i = 0; i < 3; i++) {
            if (valid[i]) {
                *out = copies[i];
                return 0;
            }
        }
    }

    // Majority voting: find two copies that agree on sequence number
    for (int i = 0; i < 3; i++) {
        if (!valid[i]) continue;
        for (int j = i + 1; j < 3; j++) {
            if (!valid[j]) continue;
            if (copies[i].sequence == copies[j].sequence) {
                *out = copies[i];
                return 0;
            }
        }
    }

    // No majority — fall back to highest sequence number among valid copies
    metadata_t *best = NULL;
    for (int i = 0; i < 3; i++) {
        if (!valid[i]) continue;
        if (best == NULL || copies[i].sequence > best->sequence) {
            best = &copies[i];
        }
    }

    *out = *best;
    return 0;
}