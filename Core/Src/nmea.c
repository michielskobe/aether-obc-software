/**
  ******************************************************************************
  * @file           : nmea.c
  * @brief          : Implementation for nmea.h
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
#include "nmea.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* Extract a field from a comma-separated NMEA sentence */
static int nmea_get_field(const char *sentence, uint8_t field, char *out, uint8_t out_size)
{
    // Skip to the start of the requested field
    uint8_t current = 0;
    while (*sentence != '\0' && current < field) {
        if (*sentence == ',') {
            current++;
        }
        sentence++;
    }
        
    // Copy until next comma, '*', or end of string
    uint8_t i = 0;
    while (*sentence != '\0' && *sentence != ',' && *sentence != '*' && i < out_size - 1) {
        out[i++] = *sentence++;
    }
    out[i] = '\0';
    return 0;
}

/* Parse latitude or longitude from NMEA format (DDMM.MMMMM or DDDMM.MMMMM) */
static uint32_t nmea_parse_latlon(const char *field, char hemi)
{
    /* Locate the decimal point to determine how many digits belong to degrees.
     * Latitude:  DDMM.MMMMM  → 2 degree digits
     * Longitude: DDDMM.MMMMM → 3 degree digits                               
     */

    if (!field || strlen(field) == 0) {
        return 0; // No fix
    }

    const char *dot = strchr(field, '.');
    if (!dot) {
        return 0; // Invalid format, no decimal point
    }
 
    // Degrees are everything before the MM pair before the dot
    int deg_len = (int)(dot - field) - 2;
    if (deg_len <= 0 || deg_len > 3) {
        return 0; // Invalid format, unexpected number of degree digits
    }

    int32_t degrees = 0;
    for (int i = 0; i < deg_len; i++) {
        if (field[i] < '0' || field[i] > '9') {
            return 0; // Invalid format, non-digit character in degree part
        }
        degrees = degrees * 10 + (field[i] - '0');
    }

    // Minutes start 2 chars before the dot
    const float minutes = strtof(field + deg_len, NULL);
    if (minutes < 0.0f || minutes >= 60.0f) {
        return 0; // Invalid format, minutes out of range
    }

    // Combine degrees and minutes (South and West hemispheres are negative)
    float value = (degrees + minutes / 60.0f);
    if (hemi == 'S' || hemi == 'W') {
        value = -value;
    }

    if (deg_len == 3) {
        /* Longitude: -180..+180 → scaled to 2^24 */
        return (uint32_t)((value + 180.0f) * (16777216.0f / 360.0f) + 0.5f) & 0xFFFFFF;
    } else {
        /* Latitude: -90..+90 → scaled to 2^24 */
        return (uint32_t)((value + 90.0f)  * (16777216.0f / 180.0f) + 0.5f) & 0xFFFFFF;
    }
}

/* Compute checksum for an NMEA sentence */
static uint8_t nmea_compute_checksum(const char *sentence)
{
    uint8_t cs = 0;

    // Skip '$'
    if (*sentence == '$') sentence++;

    // XOR all characters until '*' or end of string
    while (*sentence != '*' && *sentence != '\0') {
        cs ^= (uint8_t)(*sentence++);
    }

    return cs;
}

/* Validate checksum for an NMEA sentence */
static int nmea_validate_checksum(const char *sentence)
{
    // Find the '*' character that precedes the checksum
    const char *star = strchr(sentence, '*');
    if (!star || !*(star + 1)) return -1;

    // Convert the received checksum from hex string to integer
    uint8_t received = (uint8_t)strtol(star + 1, NULL, 16);

    // Compute the checksum of the sentence
    uint8_t computed = nmea_compute_checksum(sentence);

    // Compare and return result
    return received == computed ? 0 : -1;
}

/* Parse GGA sentence */
static void parse_gga(const char *s, nmea_data_t *nmea_data_packet)
{
    // More info about GGA sentence can be found here: https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html

    // Temporary buffers for field parsing
    char buf[16] = {0};
    char hemi[2] = {0};
 
    /* Latitude */
    if (nmea_get_field(s, 2, buf, sizeof(buf)) == 0 && nmea_get_field(s, 3, hemi, sizeof(hemi)) == 0){
        nmea_data_packet->latitude = nmea_parse_latlon(buf, hemi[0]);
    }
 
    /* Longitude */
    if (nmea_get_field(s, 4, buf, sizeof(buf)) == 0 && nmea_get_field(s, 5, hemi, sizeof(hemi)) == 0) {
        nmea_data_packet->longitude = nmea_parse_latlon(buf, hemi[0]);
    }
 
    /* Fix quality */
    if (nmea_get_field(s, 6, buf, sizeof(buf)) == 0) {
        nmea_data_packet->fix_quality = (uint8_t)atoi(buf);
    }
 
    /* Number of satellites */
    if (nmea_get_field(s, 7, buf, sizeof(buf)) == 0) {
        nmea_data_packet->satellites = (uint8_t)atoi(buf);
    }

    /* Horizontal Dilution of Precision */
    if (nmea_get_field(s, 8, buf, sizeof(buf)) == 0) {
        nmea_data_packet->hdop_x10 = (uint8_t)(strtof(buf, NULL) * 10); // Scale by 10 and convert to integer
    }
 
    /* Altitude above mean sea level */
    if (nmea_get_field(s, 9, buf, sizeof(buf)) == 0) {
        nmea_data_packet->altitude = (uint16_t)(atof(buf) / 1.5 + 0.5);
    }
}

/* Public user code ----------------------------------------------------------*/

int nmea_parse_sentence(const char *sentence, nmea_data_t *nmea_data_packet)
{
    // The sentence type starts at character 1 (skip '$')
    if (!sentence || sentence[0] != '$') {
        return -1; // Invalid sentence
    }

    // Validate checksum first
    if (nmea_validate_checksum(sentence) != 0) {
        return -1; // Checksum error
    }
     
    // Skip the $-character and the  two-character talker ID
    const char *msg = sentence + 3;
 
    if (strncmp(msg, "GGA", 3) == 0) {
        parse_gga(sentence, nmea_data_packet);
        return 0; // Successfully parsed GGA sentence
    }
    // Other parsers can be easily added if wanted
    // Unknown sentences are silently ignored
    return 1; // Unknown sentence type, not an error but no data parsed
}