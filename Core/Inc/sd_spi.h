/**
  ******************************************************************************
  * @file           : sd_spi.h
  * @brief          : Header for sd_spi.c file.
  *                   This file contains the defines for the SD SPI functionality.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SD_SPI_H
#define __SD_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "data_packet.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define SD_BLOCK_SIZE 512

/* Exported functions prototypes ---------------------------------------------*/

/**
* @brief Initialize the SD card in SPI mode.
*
* Performs the full SD card initialization sequence required for SPI mode,
* including:
*   - Sending initial clock pulses with CS high
*   - CMD0 (GO_IDLE_STATE)
*   - CMD8 (SEND_IF_COND) to check voltage range (for SD v2+)
*   - ACMD41 (SD_SEND_OP_COND) to initialize the card
*   - CMD58 (READ_OCR) to verify power-up status
*
* The function configures the card into a ready state for block
* read/write operations.
*
* @return int
*         - 0 on success (card initialized and ready)
*         - Negative value on failure
*
* @note This function assumes:
*       - SPI is initialized at a low frequency (<400 kHz) for initialization
*       - GPIO and CS pin are properly configured
*
* @warning After successful initialization, the SPI clock should typically
*          be increased for normal operation.
*
*/
int sd_init(void);

/**
* @brief Write a single 512-byte block to the SD card.
*
* Sends a CMD24 (WRITE_BLOCK) command and writes exactly one block
* (512 bytes) of data to the specified block address.
*
* @param[in] block_addr  Logical block address (LBA).
*                        - For SDSC cards: byte address (block_addr * 512)
*                        - For SDHC/SDXC cards: block address directly
*
* @param[in] data        Pointer to a 512-byte buffer containing the data to write.
*
* @return int
*         - 0 on success (block written correctly)
*         - Negative value on failure
*
* @see transmit_data_packet()
*
*/
int sd_write_block(uint32_t block_addr, uint8_t *data);

/**
* @brief Write multiple 512-byte blocks to the SD card (CMD25).
*
* This function performs a multi-block write operation using CMD25
* (WRITE_MULTIPLE_BLOCK). It writes a sequence of contiguous blocks
* starting at the specified address.
*
* @param[in] start_block_addr  Starting logical block address (LBA).
*                             - SDSC: byte address (addr * 512)
*                             - SDHC/SDXC: block address directly
*
* @param[in] data             Pointer to the data buffer.
*                             Must contain (number_of_blocks * 512) bytes.
*
* @param[in] number_of_blocks  Number of 512-byte blocks to write.
*
* @return int
*         - 0 on success (all blocks written)
*         - Negative value on failure
*
* @see transmit_data_packet()
*
*/
int sd_write_multiple_block(uint32_t start_block_addr, uint8_t *data, uint8_t number_of_blocks);

/**
* @brief Read a single 512-byte block from the SD card.
*
* Sends a CMD17 (READ_SINGLE_BLOCK) command and reads exactly one
* 512-byte block from the specified block address.
*
* @param[in]  block_addr  Logical block address (LBA).
*                         - For SDSC cards: byte address (block_addr * 512)
*                         - For SDHC/SDXC cards: block address directly
*
* @param[out] buffer      Pointer to a 512-byte buffer where the read data
*                         will be stored.
*
* @return int
*         - 0 on success (block read correctly)
*         - Negative value on failure
*
* @note The function waits for a start block token (0xFE) before reading data.
*
* @note After the data block, 2 CRC bytes are read and discarded.
*
* @see read_data_packet()
*
*/
int sd_read_block(uint32_t block_addr, uint8_t *buffer);

/**
* @brief Read multiple 512-byte blocks from the SD card.
*
* This function performs a multi-block read operation, 
* starting at the specified address.
*
* @param[in] start_block_addr  Starting logical block address (LBA).
*                             - SDSC cards: byte address (addr * 512)
*                             - SDHC/SDXC cards: block address directly
*
* @param[out] buffer          Pointer to buffer where the read data will be stored.
*                             Must be suited for (number_of_blocks * 512) bytes.
*
* @param[in] number_of_blocks  Number of 512-byte blocks to read.
*
* @return int
*         - 0 on success
*         - Negative value on failure
*
* @see sd_read_block()
*
*/
int sd_read_multiple_block(uint32_t start_block_addr, uint8_t *buffer, uint8_t number_of_blocks);

/** 
 * @brief Write a metadata structure to the SD card, with CRC validation and triple redundancy.
 *
* This function writes the provided metadata structure to the first three sectors of the SD card 
 * (blocks 0, 1, and 2) to ensure redundancy. Each sector will contain an identical copy of the 
 * metadata along with a CRC for integrity verification.
 * 
 * @param[in] metadata Pointer to the metadata structure to be written.
 * 
 * @return int
 *         - 0 on success (all copies written successfully)
 *         - Negative value on failure (if any copy fails to write)
 */
int metadata_write(metadata_t *metadata);

/**
 * @brief Read the metadata structure from the SD card.
 * 
 * This function reads the metadata from the first three sectors of the SD card (blocks 0, 1, and 2),
 * checks the CRC for each copy, and returns the first valid copy it finds. If multiple
 * valid copies are found, it can implement a strategy to determine which one to return (e.g., majority voting).
 *
 * @param[out] out Pointer to the metadata structure where the read data will be stored.
 *
 * @return int
 *         - 0 on success (metadata read correctly)
 *         - Negative value on failure
 */
int metadata_read(metadata_t *out);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __SD_SPI_H */