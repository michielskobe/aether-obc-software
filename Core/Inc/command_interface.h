#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

void CommandInterface_ProcessMessage(const can_rx_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_INTERFACE_H */