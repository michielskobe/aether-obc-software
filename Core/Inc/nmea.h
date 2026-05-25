/**
  ******************************************************************************
  * @file           : nmea.h
  * @brief          : Header for nmea.c file.
  *                   This file contains the defines for the NMEA message parsing functionality.
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
#ifndef __NMEA_H
#define __NMEA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* NMEA data packet */
typedef struct {
    uint32_t latitude;      // Latitude, encoded to 24-bit unsigned fixed-point (-90..+90 → 0..16777215)
    uint32_t longitude;     // Longitude, encoded to 24-bit unsigned fixed-point (-180..+180 → 0..16777215)
    uint16_t altitude;      // Altitude, encoded to 16-bit unsigned (1.5 m/step, max ~98.3 km)
    uint8_t fix_quality;    // GPS fix type (0–6, per NMEA GGA standard)
    uint8_t satellites;     // Number of satellites in view (0–24+)
    uint8_t hdop_x10;       // HDOP scaled by 10 (e.g. 1.2 → 12)
} nmea_data_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Parse a single NMEA sentence. Returns:
 *  0 = parsed successfully
 *  1 = unknown sentence (ignored)
 * -1 = invalid or checksum error
 */
int nmea_parse_sentence(const char *sentence, nmea_data_t *nmea_data_packet);

#ifdef __cplusplus
}
#endif

#endif /* __NMEA_H */