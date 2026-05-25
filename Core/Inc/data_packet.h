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

#ifdef __cplusplus
}
#endif

#endif /* DATA_PACKET_H */