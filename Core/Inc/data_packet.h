#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t timestamp[3];
    uint8_t id;
    uint8_t data[4];
} data_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t last_written_sector;
    uint8_t rxsm_lo;
    uint8_t rxsm_sods;
    uint8_t rxsm_soe;
    uint8_t ffu_ejection;
    uint8_t cgg1_fired;
    uint8_t cgg2_fired;
    uint8_t bw1_fired;
    uint8_t bw2_fired;
    uint8_t gnss[8];
    uint16_t crc;
} metadata_t;

#ifdef __cplusplus
}
#endif

#endif /* DATA_PACKET_H */