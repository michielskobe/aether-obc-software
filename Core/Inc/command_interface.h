#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Dispatch an incoming CAN message to the appropriate handler.
 *         Also clears any pending-retry entry for the arriving reply ID.
 *         Call from the CommandInterface task whenever a message is dequeued.
 */

void CommandInterface_ProcessMessage(const can_rx_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_INTERFACE_H */