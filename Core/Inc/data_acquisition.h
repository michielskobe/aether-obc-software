#ifndef DATA_ACQUISITION_H
#define DATA_ACQUISITION_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

void DataAcquisition_ProcessMessage(const can_rx_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* DATA_ACQUISITION_H */